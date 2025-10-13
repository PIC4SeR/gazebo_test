"""PostgreSQL-backed checkpoint store for experiment orchestration.

This module exposes a concurrency-safe API to register and claim experiment
runs while keeping a durable, single source of truth that multiple workers can
share. PostgreSQL is used as the storage backend so that jobs can be safely
coordinated across machines.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
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
                        updated_at
                    ) VALUES (%(algorithm)s, %(experiment_identifier)s, %(experiment_tag)s, %(run_id)s,
                              'PENDING', %(ts)s, %(ts)s)
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
                           result = COALESCE(result, %(note)s)
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
                           updated_at = %(ts)s
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
            statuses = ("PENDING", "RUNNING")
        else:
            statuses = ("PENDING",)
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
                           updated_at = %(ts)s
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
                           updated_at = %(ts)s
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
                           updated_at = %(ts)s
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

    @staticmethod
    def _timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()
