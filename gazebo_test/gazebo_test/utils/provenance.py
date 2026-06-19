"""Experiment provenance: tie a group of results to one config + controller.

Writes, into the results base directory:

  * a verbatim copy of the navigation config file actually used (the YAML
    rendered from the navigator profile templates), and
  * the git commit (and dirty flag) of the one controller package employed,

so we can be sure every run in a group used the same parameters and the same
controller implementation.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Optional

import yaml

PROVENANCE_FILENAME = "experiment_provenance.yaml"


def _run_git(repo: Path, args) -> Optional[str]:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo), *args],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except Exception:  # noqa: BLE001 - git missing/slow/unavailable
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def find_workspace_src(anchors) -> Optional[Path]:
    """Locate the workspace ``src`` directory by walking up from ``anchors``."""
    env_src = os.environ.get("EXPERIMENT_WORKSPACE_SRC")
    if env_src and Path(env_src).is_dir():
        return Path(env_src).resolve()
    env_ws = os.environ.get("EXPERIMENT_WORKSPACE")
    if env_ws and (Path(env_ws) / "src").is_dir():
        return (Path(env_ws) / "src").resolve()

    for anchor in anchors:
        try:
            anchor = Path(anchor).resolve()
        except Exception:  # noqa: BLE001
            continue
        for parent in [anchor, *anchor.parents]:
            src = parent / "src"
            if src.is_dir() and (
                (parent / "install").is_dir()
                or (parent / "build").is_dir()
                or any(src.glob("*/.git"))
            ):
                return src.resolve()
    return None


def resolve_controller(config_path: Path) -> Dict[str, object]:
    """Extract the controller plugin/package employed from a nav2 config YAML."""
    info: Dict[str, object] = {}
    try:
        with config_path.open("r") as handle:
            data = yaml.safe_load(handle) or {}
    except Exception as exc:  # noqa: BLE001
        return {"error": f"could not parse config: {exc}"}

    params = (data.get("controller_server") or {}).get("ros__parameters") or {}
    names = params.get("controller_plugins") or []
    if not names:
        return {"error": "no controller_plugins found in config"}

    # Conventionally a single FollowPath controller; use the first declared one.
    name = names[0]
    spec = params.get(name) or {}
    plugin = spec.get("plugin")
    python_class = spec.get("python_class")
    if plugin:
        info["plugin"] = plugin
    if python_class:
        info["python_class"] = python_class

    # Derive the owning package: a pybind controller names it in python_class;
    # a C++ plugin uses the "<package>::Class" convention.
    package = None
    if python_class:
        package = str(python_class).split(".", 1)[0]
    elif plugin:
        package = re.split(r"[:/]", str(plugin), 1)[0]
    if package:
        info["package"] = package
    return info


def find_package_source(package: str, src: Path) -> Optional[Path]:
    """Find the source directory of ``package`` by matching package.xml <name>."""
    for pkg_xml in src.rglob("package.xml"):
        try:
            text = pkg_xml.read_text()
        except Exception:  # noqa: BLE001
            continue
        match = re.search(r"<name>\s*([^<]+?)\s*</name>", text)
        if match and match.group(1).strip() == package:
            return pkg_xml.parent
    return None


def enclosing_git_repo(path: Path) -> Optional[Path]:
    for parent in [path, *path.parents]:
        if (parent / ".git").exists():
            return parent
    return None


def collect_controller_git(controller: Dict[str, object], src: Optional[Path]) -> Dict[str, object]:
    """Resolve the git commit of the controller package's source repo."""
    package = controller.get("package")
    if not package:
        return {"error": "controller package could not be determined"}
    if src is None:
        return {"error": "workspace src not found; cannot locate controller source"}
    pkg_dir = find_package_source(str(package), src)
    if pkg_dir is None:
        return {"error": f"package '{package}' not found under {src}"}
    repo = enclosing_git_repo(pkg_dir)
    if repo is None:
        return {"error": f"no git repository found for package '{package}'"}
    commit = _run_git(repo, ["rev-parse", "HEAD"])
    if commit is None:
        return {"error": f"could not read git HEAD for '{package}'"}
    status = _run_git(repo, ["status", "--porcelain"])
    return {
        "repo": str(repo),
        "commit": commit,
        "branch": _run_git(repo, ["rev-parse", "--abbrev-ref", "HEAD"]),
        "dirty": bool(status),
    }


def write_experiment_provenance(
    *,
    base_path: Path,
    algorithm_name: str,
    experiment_identifier: str,
    config_file: Optional[str] = None,
    anchor_paths=None,
    extra: Optional[Dict[str, object]] = None,
    logger=None,
) -> Optional[Path]:
    """Write the provenance manifest (and copy the config) into ``base_path``.

    Best-effort: never raises. Returns the manifest path on success or ``None``.
    """

    def _log(level: str, message: str) -> None:
        if logger is None:
            return
        getattr(logger, level, logger.info)(message)

    try:
        base_path = Path(base_path)
        base_path.mkdir(parents=True, exist_ok=True)

        manifest: Dict[str, object] = {
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "algorithm_name": algorithm_name,
            "experiment_identifier": experiment_identifier,
        }
        if extra:
            manifest.update(extra)

        controller: Dict[str, object] = {}

        # --- copy the (templated) config file actually used ----------------
        if config_file:
            source = Path(config_file).expanduser()
            if source.is_file():
                copied_name = "nav_params_used" + (source.suffix or ".yaml")
                shutil.copyfile(source, base_path / copied_name)
                manifest["config_file"] = copied_name
                controller = resolve_controller(source)
            else:
                manifest["config_file"] = None
                controller = {"error": "config file not found"}
                _log("warning", f"Provenance: config file not found: {source}")

        # --- git commit of the one controller package employed -------------
        src = find_workspace_src(list(anchor_paths or []) + [Path(__file__), Path.cwd()])
        controller["git"] = collect_controller_git(controller, src)
        manifest["controller"] = controller

        # --- atomic write --------------------------------------------------
        manifest_path = base_path / PROVENANCE_FILENAME
        tmp_path = manifest_path.with_suffix(f".{os.getpid()}.tmp")
        with tmp_path.open("w") as handle:
            yaml.safe_dump(manifest, handle, sort_keys=False, default_flow_style=False)
        tmp_path.replace(manifest_path)
        _log("info", f"Experiment provenance written to {manifest_path}")
        return manifest_path
    except Exception as exc:  # noqa: BLE001 - provenance must never break a run
        _log("warning", f"Failed to write experiment provenance: {exc}")
        return None
