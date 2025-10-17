"""PostgreSQL-backed checkpoint store for experiment orchestration.

This module exposes a concurrency-safe API to register and claim experiment
runs while keeping a durable, single source of truth that multiple workers can
share. PostgreSQL is used as the storage backend so that jobs can be safely
coordinated across machines.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone, timedelta
from typing import Dict, Iterable, List, Optional, Sequence

import psycopg
from psycopg.rows import dict_row
from psycopg import sql


@dataclass(frozen=True)
class ExperimentJobDescriptor:
    """Descriptor for a unique experiment configuration."""

    algorithm: str
    experiment_identifier: str
    experiment_tag: str
    run_id: int


@dataclass
class ExperimentJob:
    """Concrete job persisted in the database."""

    id: int
    algorithm: str
    experiment_identifier: str
    experiment_tag: str
    run_id: int
    status: str
    result: Optional[str]
    worker_id: Optional[str]
    updated_at: str

    def descriptor(self) -> ExperimentJobDescriptor:
        return ExperimentJobDescriptor(
            algorithm=self.algorithm,
            experiment_identifier=self.experiment_identifier,
            experiment_tag=self.experiment_tag,
            run_id=self.run_id,
        )


class ExperimentCheckpointStore:
    """PostgreSQL-backed store that tracks experiment job state."""

    def __init__(
        self,
        dsn: str,
        *,
        connect_timeout_sec: float = 5.0,
    ) -> None:
        if not dsn:
            raise ValueError(
                "A PostgreSQL DSN must be provided to initialise the checkpoint store"
            )
        self._dsn = dsn
        self._connect_timeout = int(max(connect_timeout_sec, 0.0))
        self._ensure_schema()

    # --- public API -----------------------------------------------------
    def register_jobs(self, jobs: Iterable[ExperimentJobDescriptor]) -> None:
        """Insert job descriptors into the database if they are missing."""

        rows = [
            {
                "algorithm": descriptor.algorithm,
                "experiment_identifier": descriptor.experiment_identifier,
                "experiment_tag": descriptor.experiment_tag,
                "run_id": descriptor.run_id,
                "ts": self._timestamp(),
            }
            for descriptor in jobs
        ]
        if not rows:
            return
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.executemany(
                    """
                    INSERT INTO jobs (
                        algorithm,
                        experiment_identifier,
                        experiment_tag,
                        run_id,
                        status,
                        created_at,
                        updated_at,
                        heartbeat_at
                    ) VALUES (%(algorithm)s, %(experiment_identifier)s, %(experiment_tag)s, %(run_id)s,
                              'PENDING', %(ts)s, %(ts)s, NULL)
                    ON CONFLICT (algorithm, experiment_identifier, experiment_tag, run_id)
                    DO NOTHING
                    """,
                    rows,
                )

    def recover_stale_jobs(self, *, worker_hint: Optional[str] = None) -> int:
        """Mark jobs that were running as failed to recover from crashes.

        Args:
            worker_hint: Optional identifier added to recovery logs.

        Returns:
            Number of jobs transitioned from RUNNING to FAILED.
        """

        recovery_note = "Recovered after unclean shutdown"
        if worker_hint:
            recovery_note += f" ({worker_hint})"
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE jobs
                       SET status = 'FAILED',
                           updated_at = %(ts)s,
                           result = COALESCE(result, %(note)s),
                           heartbeat_at = NULL
                     WHERE status = 'RUNNING'
                    """,
                    {"ts": self._timestamp(), "note": recovery_note},
                )
                return cur.rowcount or 0

    def claim_next_job(
        self,
        *,
        algorithm: str,
        experiment_identifier: str,
        worker_id: str,
    ) -> Optional[ExperimentJob]:
        """Atomically claim the next PENDING job for execution."""

        with psycopg.connect(
            self._dsn,
            autocommit=False,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor(row_factory=dict_row) as cur:
                cur.execute(
                    """
                    SELECT id, algorithm, experiment_identifier, experiment_tag, run_id,
                           status, result, worker_id, updated_at
                      FROM jobs
                     WHERE algorithm = %(algorithm)s
                       AND experiment_identifier = %(experiment_identifier)s
                       AND status = 'PENDING'
                  ORDER BY run_id, id
                     FOR UPDATE SKIP LOCKED
                     LIMIT 1
                    """,
                    {
                        "algorithm": algorithm,
                        "experiment_identifier": experiment_identifier,
                    },
                )
                row = cur.fetchone()
                if row is None:
                    conn.rollback()
                    return None
                job_id = row["id"]
                now_ts = self._timestamp()
                cur.execute(
                    """
                    UPDATE jobs
                       SET status = 'RUNNING',
                           worker_id = %(worker_id)s,
                           updated_at = %(ts)s,
                           heartbeat_at = %(ts)s
                     WHERE id = %(job_id)s
                    """,
                    {"worker_id": worker_id, "ts": now_ts, "job_id": job_id},
                )
                conn.commit()
        row.update({"status": "RUNNING", "worker_id": worker_id, "updated_at": now_ts})
        return ExperimentJob(**row)

    def get_job(self, descriptor: ExperimentJobDescriptor) -> Optional[ExperimentJob]:
        params: Dict[str, object] = {
            "algorithm": descriptor.algorithm,
            "experiment_identifier": descriptor.experiment_identifier,
            "experiment_tag": descriptor.experiment_tag,
            "run_id": descriptor.run_id,
        }
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor(row_factory=dict_row) as cur:
                cur.execute(
                    """
                    SELECT id, algorithm, experiment_identifier, experiment_tag, run_id,
                           status, result, worker_id, updated_at
                      FROM jobs
                     WHERE algorithm = %(algorithm)s
                       AND experiment_identifier = %(experiment_identifier)s
                       AND experiment_tag = %(experiment_tag)s
                       AND run_id = %(run_id)s
                    """,
                    params,
                )
                row = cur.fetchone()
        if row is None:
            return None
        return ExperimentJob(**row)

    def has_incomplete_jobs(
        self,
        *,
        algorithm: str,
        experiment_identifier: str,
        include_running: bool = True,
    ) -> bool:
        """Return True when any job still needs work.

        Args:
            algorithm: Algorithm identifier.
            experiment_identifier: Experiment identifier.
            include_running: When True (default), treat RUNNING jobs as incomplete.

        Returns:
            True if a matching job has status PENDING (or RUNNING when requested).
        """

        statuses: Sequence[str]
        if include_running:
            statuses = ("PENDING", "RUNNING", "FAILED")
        else:
            statuses = ("PENDING", "FAILED")
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    SELECT 1
                      FROM jobs
                     WHERE algorithm = %(algorithm)s
                       AND experiment_identifier = %(experiment_identifier)s
                       AND status = ANY(%(statuses)s)
                     LIMIT 1
                    """,
                    {
                        "algorithm": algorithm,
                        "experiment_identifier": experiment_identifier,
                        "statuses": list(statuses),
                    },
                )
                return cur.fetchone() is not None

    def refresh_job_heartbeat(self, job_id: int, worker_id: str) -> bool:
        """Update the heartbeat timestamp for a RUNNING job.

        Returns True when the heartbeat was updated; False if the job is no longer
        owned by the worker (e.g. requeued or finished).
        """

        now_ts = self._timestamp()
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE jobs
                       SET heartbeat_at = %(ts)s,
                           updated_at = %(ts)s
                     WHERE id = %(job_id)s
                       AND worker_id = %(worker_id)s
                       AND status = 'RUNNING'
                    """,
                    {"ts": now_ts, "job_id": job_id, "worker_id": worker_id},
                )
                return cur.rowcount == 1

    def requeue_stale_jobs(
        self,
        *,
        algorithm: str,
        experiment_identifier: str,
        heartbeat_timeout_sec: float,
    ) -> int:
        """Move RUNNING jobs without recent heartbeats back to PENDING."""

        timeout = max(heartbeat_timeout_sec, 0.0)
        threshold_dt = datetime.now(timezone.utc) - timedelta(seconds=timeout)
        threshold = threshold_dt.isoformat()
        note = (
            f"Requeued after missing heartbeat for {timeout:.1f}s"
            if timeout > 0
            else "Requeued after missing heartbeat"
        )
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE jobs
                       SET status = 'PENDING',
                           worker_id = NULL,
                           result = COALESCE(result, %(note)s),
                           updated_at = %(ts)s,
                           heartbeat_at = NULL
                     WHERE algorithm = %(algorithm)s
                       AND experiment_identifier = %(experiment_identifier)s
                       AND status = 'RUNNING'
                       AND (heartbeat_at IS NULL OR heartbeat_at < %(threshold)s)
                    """,
                    {
                        "algorithm": algorithm,
                        "experiment_identifier": experiment_identifier,
                        "threshold": threshold,
                        "note": note,
                        "ts": self._timestamp(),
                    },
                )
                return cur.rowcount or 0

    def force_requeue_job(
        self,
        job_id: int,
        *,
        reason: Optional[str] = None,
        expected_worker_id: Optional[str] = None,
    ) -> bool:
        """Force a RUNNING job back to PENDING immediately.

        Args:
            job_id: Identifier of the job to requeue.
            reason: Optional note to persist as the job result.
            expected_worker_id: When provided, only requeue the job if the
                worker currently assigned matches this value.

        Returns:
            True when the job was transitioned to PENDING, False otherwise.
        """

        note = reason or "Requeued by watchdog request"
        params: Dict[str, object] = {
            "job_id": job_id,
            "note": note,
            "ts": self._timestamp(),
        }
        worker_clause = ""
        if expected_worker_id is not None:
            params["worker_id"] = expected_worker_id
            worker_clause = " AND worker_id = %(worker_id)s"
        query = (
            """
            UPDATE jobs
               SET status = 'PENDING',
                   worker_id = NULL,
                   result = COALESCE(result, %(note)s),
                   updated_at = %(ts)s,
                   heartbeat_at = NULL
             WHERE id = %(job_id)s
               AND status = 'RUNNING'
            """
            + worker_clause
        )
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(query, params)
                return cur.rowcount == 1

    def mark_job_done(self, job_id: int, result: str) -> None:
        """Mark a job as successfully completed."""

        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE jobs
                       SET status = 'DONE',
                           result = %(result)s,
                           updated_at = %(ts)s,
                           heartbeat_at = NULL
                     WHERE id = %(job_id)s
                    """,
                    {"result": result, "ts": self._timestamp(), "job_id": job_id},
                )

    def mark_job_failed(self, job_id: int, error_message: str) -> None:
        """Mark a job as failed with the given error description."""

        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE jobs
                       SET status = 'FAILED',
                           result = %(msg)s,
                           updated_at = %(ts)s,
                           heartbeat_at = NULL
                     WHERE id = %(job_id)s
                    """,
                    {"msg": error_message, "ts": self._timestamp(), "job_id": job_id},
                )

    def list_jobs(
        self,
        *,
        algorithm: str,
        experiment_identifier: str,
        statuses: Optional[Sequence[str]] = None,
    ) -> List[ExperimentJob]:
        query = """
            SELECT id, algorithm, experiment_identifier, experiment_tag, run_id,
                   status, result, worker_id, updated_at
              FROM jobs
             WHERE algorithm = %(algorithm)s
               AND experiment_identifier = %(experiment_identifier)s
        """
        params: Dict[str, object] = {
            "algorithm": algorithm,
            "experiment_identifier": experiment_identifier,
        }
        if statuses:
            query += " AND status = ANY(%(statuses)s)"
            params["statuses"] = list(statuses)
        query += " ORDER BY run_id, id"
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor(row_factory=dict_row) as cur:
                cur.execute(sql.SQL(query), params)
                rows = cur.fetchall()
        return [ExperimentJob(**row) for row in rows]

    def reset_jobs(
        self,
        *,
        algorithm: str,
        experiment_identifier: str,
    ) -> int:
        """Reset all jobs to PENDING and clear results for a given descriptor."""

        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    UPDATE jobs
                       SET status = 'PENDING',
                           result = NULL,
                           worker_id = NULL,
                                                     updated_at = %(ts)s,
                                                     heartbeat_at = NULL
                     WHERE algorithm = %(algorithm)s
                       AND experiment_identifier = %(experiment_identifier)s
                    """,
                    {
                        "algorithm": algorithm,
                        "experiment_identifier": experiment_identifier,
                        "ts": self._timestamp(),
                    },
                )
                return cur.rowcount or 0

    def requeue_failed_jobs(
        self,
        *,
        algorithm: Optional[str] = None,
        experiment_identifier: Optional[str] = None,
    ) -> int:
        """Mark FAILED jobs as PENDING so they can be re-run."""

        clauses: List[sql.SQL] = [sql.SQL("status = 'FAILED'")]
        params: Dict[str, object] = {"ts": self._timestamp()}
        if algorithm is not None:
            clauses.append(sql.SQL("algorithm = %(algorithm)s"))
            params["algorithm"] = algorithm
        if experiment_identifier is not None:
            clauses.append(sql.SQL("experiment_identifier = %(experiment_identifier)s"))
            params["experiment_identifier"] = experiment_identifier
        where_clause = sql.SQL(" AND ").join(clauses)

        query = sql.SQL(
            """
            UPDATE jobs
               SET status = 'PENDING',
                   result = NULL,
                   worker_id = NULL,
                   updated_at = %(ts)s,
                   heartbeat_at = NULL
             WHERE {}
        """
        ).format(where_clause)

        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(query, params)
                return cur.rowcount or 0

    # --- internals --------------------------------------------------
    def _ensure_schema(self) -> None:
        with psycopg.connect(
            self._dsn,
            autocommit=True,
            connect_timeout=self._connect_timeout,
        ) as conn:
            with conn.cursor() as cur:
                cur.execute(
                    """
                    CREATE TABLE IF NOT EXISTS jobs (
                        id SERIAL PRIMARY KEY,
                        algorithm TEXT NOT NULL,
                        experiment_identifier TEXT NOT NULL,
                        experiment_tag TEXT NOT NULL,
                        run_id INTEGER NOT NULL,
                        status TEXT NOT NULL CHECK (status IN ('PENDING', 'RUNNING', 'DONE', 'FAILED')),
                        result TEXT,
                        worker_id TEXT,
                        created_at TIMESTAMPTZ NOT NULL,
                        updated_at TIMESTAMPTZ NOT NULL,
                        heartbeat_at TIMESTAMPTZ,
                        UNIQUE (algorithm, experiment_identifier, experiment_tag, run_id)
                    )
                    """,
                )
                cur.execute(
                    """
                    CREATE INDEX IF NOT EXISTS idx_jobs_pending
                        ON jobs (algorithm, experiment_identifier, run_id)
                     WHERE status = 'PENDING'
                    """,
                )
                cur.execute(
                    """
                    ALTER TABLE jobs
                    ADD COLUMN IF NOT EXISTS heartbeat_at TIMESTAMPTZ
                    """,
                )
                cur.execute(
                    """
                    CREATE INDEX IF NOT EXISTS idx_jobs_running_heartbeat
                        ON jobs (algorithm, experiment_identifier, heartbeat_at)
                     WHERE status = 'RUNNING'
                    """,
                )

    @staticmethod
    def _timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()
