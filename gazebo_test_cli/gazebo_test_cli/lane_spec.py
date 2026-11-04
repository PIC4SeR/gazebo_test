"""What a lane runs: the commands, environment and order of one experiment batch.

:func:`build_lane_spec` turns the ``ros2 gazeboexp run`` arguments into a
:class:`LaneSpec` without starting anything. ``full_experiment`` runs a spec in
a tmux session (one window per :class:`LaneWindow`); other backends (Docker
Compose, one service per window) consume the same spec, so every backend runs
the same commands.
"""

import argparse
import os
import shlex
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

from gazebo_test.script_utils.parsing_utils import parse_experiment_name
from gazebo_test.script_utils.navigator_utils import (
    get_navigator_additional_launch_files,
    get_navigator_navigation_launch,
    get_navigator_yaml,
)
from gazebo_test.utils.checkpoint_store import ExperimentCheckpointStore
from gazebo_test_cli.launch_utils import swarm_algorithm_name


def isolate_experiment(ros_domain_id: int) -> Tuple[int, str]:
    """
    Isolate the experiment by setting the ROS_DOMAIN_ID.
    This is useful to avoid conflicts with other ROS nodes.
    """
    if ros_domain_id < 0 or ros_domain_id > 1023:
        raise ValueError("ROS_DOMAIN_ID must be between 0 and 1023.")

    os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
    gazebo_uri = f"http://localhost:{ros_domain_id * 10 + 11345}"
    return ros_domain_id, gazebo_uri


def _format_launch_value(value: Any) -> str:
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def _launch_spec_to_command(spec: Any, source: str) -> Optional[str]:
    if isinstance(spec, str):
        command = spec.strip()
        return command or None

    if isinstance(spec, dict):
        package = spec.get("package", spec.get("pkg"))
        launch_file = spec.get("launch", spec.get("launch_file", spec.get("file")))
        arguments = spec.get("arguments", {})

        if not package or not launch_file:
            raise ValueError(
                f"{source} additional launch entries must define 'package' and 'launch'."
            )

        command_parts = [shlex.quote(str(package)), shlex.quote(str(launch_file))]
        if isinstance(arguments, dict):
            command_parts.extend(
                shlex.quote(f"{key}:={_format_launch_value(value)}")
                for key, value in arguments.items()
            )
        elif isinstance(arguments, list):
            command_parts.extend(shlex.quote(str(argument)) for argument in arguments)
        elif arguments:
            raise ValueError(
                f"{source} additional launch entry 'arguments' must be a mapping or list."
            )
        return " ".join(command_parts)

    raise ValueError(
        f"{source} additional launch entries must be strings or dictionaries."
    )


def _normalize_additional_launch_files(raw_launch_files: Any, source: str) -> List[str]:
    if raw_launch_files is None:
        return []
    if isinstance(raw_launch_files, (str, dict)):
        raw_launch_files = [raw_launch_files]
    if not isinstance(raw_launch_files, list):
        raise ValueError(f"{source} additional_launch_files must be a list.")

    launch_files = []
    for spec in raw_launch_files:
        command = _launch_spec_to_command(spec, source)
        if command:
            launch_files.append(command)
    return launch_files


def _normalize_navigation_launch(raw_launch_file: Any, source: str) -> Optional[str]:
    launch_files = _normalize_additional_launch_files(raw_launch_file, source)
    if len(launch_files) > 1:
        raise ValueError(f"{source} navigation_launch must define only one launch file.")
    return launch_files[0] if launch_files else None


def _dedupe_preserving_order(values: List[str]) -> List[str]:
    return list(dict.fromkeys(values))


def _launch_window_name(launch_command: str) -> str:
    try:
        command_parts = shlex.split(launch_command)
    except ValueError:
        command_parts = launch_command.split()
    if len(command_parts) >= 2:
        return Path(command_parts[1]).stem
    if command_parts:
        return Path(command_parts[0]).stem
    return "additional launch"


class _LaunchFormatValues(dict):
    def __missing__(self, key: str) -> str:
        return "{" + key + "}"


def _expand_launch_placeholders(
    launch_command: str,
    replacements: Dict[str, Any],
) -> str:
    return launch_command.format_map(_LaunchFormatValues(replacements))


def _resolve_navigation_params(
    args: argparse.Namespace, render_vars: Optional[Dict] = None
) -> str:
    navigator_path = None
    if args.nav_params:
        if args.navigator is not None:
            warnings.warn(
                "Both navigation parameters file and navigator specified. Using the navigator to generate the parameters file."
            )
            navigator_path = get_navigator_yaml(args.navigator, render_vars=render_vars)
        else:
            print(f"Using navigation parameters file: {args.nav_params} (ignoring navigator)")
            navigator_path = args.nav_params
    elif args.navigator:
        print(f"Using navigator: {args.navigator} to generate the parameters file")
        navigator_path = get_navigator_yaml(args.navigator, render_vars=render_vars)
        print(f"Generated navigation parameters file at: {navigator_path}")
    else:
        print(
            "No navigation parameters file or navigator specified. Using default nav2_params.yaml"
        )
        navigator_path = os.path.join(
            get_package_share_directory("gazebo_experiments"),
            "config",
            "nav2_params.yaml",
        )
        print(f"Using default navigation parameters file: {navigator_path}")
    return str(navigator_path)


def _default_navigation_launch_command(
    map_path: str, params_file: str, robots_arg: str = "", lone_robot: str = "jackal"
) -> str:
    cmd = f"gazebo_test bringup.launch.py map:={map_path} params_file:={params_file}"
    if robots_arg:
        # Fleet: bringup loops one namespaced stack per robot.
        cmd += " " + shlex.quote(f"robots:={robots_arg}")
    else:
        # Lone robot runs under its own namespace so it matches the now-namespaced
        # spawn / plugins.
        cmd += f" namespace:={lone_robot} use_namespace:=true"
    return cmd


def _load_goals_yaml(goals_and_poses_path: str) -> dict:
    """Load a goals/poses YAML as a dict, or {} if it can't be read/parsed."""
    try:
        with open(goals_and_poses_path, "r") as f:
            return yaml.safe_load(f) or {}
    except Exception:  # noqa: BLE001
        return {}


def _default_watchdog_nodes_for_backend(
    navigation_backend: str,
    use_evaluator: bool,
) -> Optional[str]:
    if navigation_backend != "action":
        return None
    nodes = ["/gazebo"]
    if use_evaluator:
        nodes.append("/hunav_evaluator_node")
    nodes.extend(["/hunav_agent_manager", "/hunav_plugin"])
    return ",".join(nodes)


def _normalize_navigation_backend(navigation_backend: str) -> str:
    normalized = str(navigation_backend or "nav2").strip().lower()
    aliases = {
        "navigate_to_pose": "action",
        "nav2_action": "action",
    }
    return aliases.get(normalized, normalized)


@dataclass
class LaneWindow:
    """One process of a lane: a tmux window, or a compose service."""

    role: str  # sim | nav | rviz | evaluator | extra | manager
    name: str  # tmux window name
    command: str  # shell command line, as typed into the window


@dataclass
class LaneSpec:
    """Everything one ``gazeboexp run`` starts, in start order.

    ``windows`` holds the long-running processes (Gazebo first); ``manager`` is
    the experiment manager, whose end is the end of the batch.
    """

    domain: int
    gazebo_uri: str
    env: Dict[str, str]
    windows: List[LaneWindow]
    manager: LaneWindow
    experiment: str = ""
    algorithm: str = ""
    params_file: str = ""
    notes: List[str] = field(default_factory=list)

    def window(self, role: str) -> Optional[LaneWindow]:
        return next((w for w in self.windows if w.role == role), None)


def build_lane_spec(args: argparse.Namespace) -> Optional[LaneSpec]:
    """The lane for ``args`` (a parsed ``gazeboexp run``), or None when the
    checkpoint store says every run of the experiment is already complete.

    Side effects kept from the tmux CLI: it renders the navigator's Nav2 params
    file, queries the checkpoint store, and sets ``ROS_DOMAIN_ID`` in this
    process's environment.
    """
    experiment_dict = parse_experiment_name(args.experiment)

    world = experiment_dict.get("world", "")
    map = experiment_dict.get("map", "")
    goals_and_poses = experiment_dict.get("goals_and_poses", "")
    # Task selects the experiment behaviour (go_to_pose | multirobot | ...).
    # CLI --task overrides the experiment's declared task.
    task = (args.task or experiment_dict.get("task") or "go_to_pose").strip()

    # Every goals/poses config -- single robot included -- declares a top-level
    # 'robots:' list (parse_fleet_yaml). The first robot's name doubles as its
    # namespace and (being also the model selector) must be a valid model id
    # (e.g. "jackal"); HuNav tracks it socially. For fleet tasks the whole list
    # is encoded for the launch files; the single-robot go_to_pose task keeps the
    # lone launch path (robots_arg stays empty), so the rest of the fleet -- when
    # present -- are dynamic obstacles.
    goals_yaml = _load_goals_yaml(goals_and_poses)
    all_robots = goals_yaml.get("robots") or []
    fleet_robots = all_robots if task in ("multirobot", "formation", "perturbation") else []
    robots_arg = (
        yaml.safe_dump(fleet_robots, default_flow_style=True).strip()
        if fleet_robots
        else ""
    )
    primary_robot = str(all_robots[0]["name"]) if all_robots else "jackal"
    agents_configuration_file = experiment_dict.get("agents_configuration_file", "")
    world_pkg_name = experiment_dict.get("world_pkg_name", "")
    agents_pkg_name = experiment_dict.get("agents_pkg_name", "")
    if args.no_navigation_launch and args.navigation_launch:
        raise ValueError(
            "--navigation-launch and --no-navigation-launch cannot be used together."
        )

    experiment_navigation_launch = _normalize_navigation_launch(
        experiment_dict.get("navigation_launch", None),
        f"experiment '{args.experiment}'",
    )
    navigator_navigation_launch = None
    if args.navigator:
        navigator_navigation_launch = _normalize_navigation_launch(
            get_navigator_navigation_launch(args.navigator),
            f"navigator '{args.navigator}'",
        )
    cli_navigation_launch = _normalize_navigation_launch(
        args.navigation_launch,
        "command line",
    )
    configured_navigation_launch = (
        cli_navigation_launch
        or navigator_navigation_launch
        or experiment_navigation_launch
    )

    additional_launch_files = _normalize_additional_launch_files(
        experiment_dict.get("additional_launch_files", []),
        f"experiment '{args.experiment}'",
    )
    if args.navigator:
        additional_launch_files.extend(
            _normalize_additional_launch_files(
                get_navigator_additional_launch_files(args.navigator),
                f"navigator '{args.navigator}'",
            )
        )
    additional_launch_files.extend(
        _normalize_additional_launch_files(
            args.additional_launch_files,
            "command line",
        )
    )
    additional_launch_files = _dedupe_preserving_order(additional_launch_files)

    checkpoint_dsn_value = (args.checkpoint_dsn or "").strip()
    checkpoint_enabled = bool(checkpoint_dsn_value)
    if args.disable_checkpoint:
        if checkpoint_enabled:
            print("Checkpoint store disabled by request; ignoring provided DSN.")
        checkpoint_dsn_value = ""
        checkpoint_enabled = False

    selected_algorithm_name = (args.algorithm_name or "").strip()
    if not selected_algorithm_name:
        selected_algorithm_name = (args.navigator or "").strip()
    if not selected_algorithm_name:
        selected_algorithm_name = swarm_algorithm_name(configured_navigation_launch) or ""
    if not selected_algorithm_name:
        selected_algorithm_name = "unknown"
    # Must match the identifier the experiment manager registers jobs under
    # (ExperimentManager: experiment_name_param or goals-file stem). Experiments
    # that share a goals_and_poses file (e.g. *_crowded_env all use
    # crowded_env.yaml) would otherwise collapse to the same stem here and let a
    # completed sibling run skip this experiment's launch.
    experiment_identifier = (args.experiment or Path(goals_and_poses).stem).removesuffix(
        f"_{selected_algorithm_name}"
    )

    if checkpoint_enabled:
        try:
            checkpoint_store = ExperimentCheckpointStore(checkpoint_dsn_value)
            existing_jobs = checkpoint_store.list_jobs(
                algorithm=selected_algorithm_name,
                experiment_identifier=experiment_identifier,
            )
            if existing_jobs:
                if not checkpoint_store.has_incomplete_jobs(
                    algorithm=selected_algorithm_name,
                    experiment_identifier=experiment_identifier,
                ):
                    print(
                        "All experiment runs are already completed according to the checkpoint store; skipping launch."
                    )
                    return None
        except Exception as exc:  # noqa: BLE001
            print(
                f"Warning: unable to query checkpoint store ({exc}); proceeding with full launch."
            )
            checkpoint_dsn_value = ""
            checkpoint_enabled = False

    if args.resume_checkpoint and not checkpoint_enabled:
        print(
            "--resume-checkpoint requested without an active checkpoint database; completed runs will be detected from previous outcomes files instead."
        )

    ros_domain_id, gazebo_uri = isolate_experiment(args.ros_domain_id)
    navigator_path = _resolve_navigation_params(args, experiment_dict.get("render_vars"))
    navigation_backend = _normalize_navigation_backend(
        args.navigation_backend
        or ("action" if args.no_navigation_launch or configured_navigation_launch else "nav2")
    )
    watchdog_required_nodes = args.watchdog_required_nodes
    if watchdog_required_nodes is None:
        watchdog_required_nodes = _default_watchdog_nodes_for_backend(
            navigation_backend,
            args.hunav_eval,
        )

    launch_placeholders = {
        "map": map,
        "params_file": navigator_path,
        "nav_params": navigator_path,
        "goals_and_poses": goals_and_poses,
        "experiment": args.experiment,
        "algorithm_name": selected_algorithm_name,
        "ros_domain_id": ros_domain_id,
        "gazebo_uri": gazebo_uri,
        # Fleet wiring for a swarm/formation navigation_launch override (e.g.
        # swarm_control hybrid.launch.py): the encoded robots list and its size.
        "robots": robots_arg,
        "n_robots": len(fleet_robots),
        # The lone robot's namespace (first robot of a fleet), e.g. for additional launch files.
        "namespace": primary_robot,
        # Files shipped by the experiment's own package, e.g. a per-cell params_file.
        "pkg_share": experiment_dict["pkg_share"],
    }
    navigation_launch_command = None
    if not args.no_navigation_launch:
        if configured_navigation_launch:
            navigation_launch_command = _expand_launch_placeholders(
                configured_navigation_launch,
                launch_placeholders,
            )
        else:
            navigation_launch_command = _default_navigation_launch_command(
                map,
                navigator_path,
                robots_arg,
                primary_robot,
            )
    print(f"Experiment task: {task}")
    print(f"Experiment manager navigation backend: {navigation_backend}")
    if navigation_launch_command:
        print(f"Navigation launch command: ros2 launch {navigation_launch_command}")
    if watchdog_required_nodes is not None:
        print(f"Watchdog required nodes: {watchdog_required_nodes}")

    windows: List[LaneWindow] = []
    gpu_env_var = (
        "__GLX_VENDOR_LIBRARY_NAME=nvidia __NV_PRIME_RENDER_OFFLOAD=1 "
        if not args.no_gpu
        else ""
    )
    robots_sim_arg = shlex.quote(f"robots:={robots_arg}") if robots_arg else ""
    robot_name_arg = f"robot_name:={primary_robot}" if primary_robot else ""
    windows.append(LaneWindow(
        "sim", "Gazebo",
        f"{gpu_env_var}ros2 launch gazebo_sim simulation.launch.py headless:={args.headless} world_pkg_name:={world_pkg_name} base_world:={world} \
                agents_configuration_file:={agents_configuration_file} config_pkg_name:={agents_pkg_name} \
                    use_lidar_gpu:={not args.no_gpu} {robot_name_arg} {robots_sim_arg}",
    ))

    if navigation_launch_command:
        windows.append(LaneWindow("nav", "Navigation", f"ros2 launch {navigation_launch_command}"))

    if not args.no_rviz:
        # Lone robot: the stack runs namespaced (see
        # _default_navigation_launch_command), so RViz must follow it.
        rviz_robots_arg = (
            shlex.quote(f"robots:={robots_arg}")
            if robots_arg
            else f"namespace:={primary_robot}"
        )
        windows.append(LaneWindow(
            "rviz", "RViz", f"ros2 launch gazebo_test rviz.launch.py {rviz_robots_arg}"))

    if args.hunav_eval:
        windows.append(LaneWindow(
            "evaluator", "Hunav Eval",
            f"ros2 launch hunav_evaluator hunav_evaluator_launch.py \
                metrics_file:={args.evaluation_metrics_file}",
        ))

    for launch_file in additional_launch_files:
        launch_file = _expand_launch_placeholders(launch_file, launch_placeholders)
        windows.append(LaneWindow(
            "extra", _launch_window_name(launch_file), f"ros2 launch {launch_file}"))

    base_path_for_results = (
        f"base_path:={args.base_path_for_results}"
        if args.base_path_for_results
        else ""
    )
    algorithm_name = f"algorithm_name:={selected_algorithm_name}"

    exp_config_pkg = (
        f"exp_config_pkg:={args.exp_config_pkg}" if args.exp_config_pkg else ""
    )
    wait_before_start = (
        f"wait_before_start:={args.wait_before_start}"
        if args.wait_before_start
        else ""
    )
    repetitions = (
        f"repetitions:={args.repetitions}" if args.repetitions is not None else ""
    )
    checkpoint_arg = ""
    notes: List[str] = []
    if checkpoint_enabled and checkpoint_dsn_value:
        checkpoint_arg = f"checkpoint_dsn:={checkpoint_dsn_value}"
        notes.append("Using PostgreSQL checkpoint store via provided DSN")

    resume_checkpoint = (
        f"resume_checkpoint:={str(args.resume_checkpoint).lower()}"
        if args.resume_checkpoint is not None
        else ""
    )
    navigation_backend_arg = f"navigation_backend:={navigation_backend}"
    watchdog_required_nodes_arg = (
        f"watchdog_required_nodes:={watchdog_required_nodes}"
        if watchdog_required_nodes is not None
        else ""
    )
    # Record the navigation config used in the results provenance.
    params_file_arg = (
        f"params_file:={navigator_path}" if navigator_path else ""
    )
    # An experiment may declare its own episode budget (sim seconds); the
    # manager otherwise falls back to experiment_config.yaml's.
    timeout_arg = (
        f"timeout_duration:={float(experiment_dict['timeout_duration'])}"
        if experiment_dict.get("timeout_duration") else ""
    )
    # ... and its wall-clock stall guard, for scenes expected to run slow.
    stall_arg = (
        f"stall_factor:={float(experiment_dict['stall_factor'])}"
        if experiment_dict.get("stall_factor") else ""
    )
    episodes_arg = f"episodes:={args.episodes}" if args.episodes else ""
    # Large fleets record a subset of each robot's topics (experiment key).
    bag_topics_arg = (
        f"bag_robot_topics:={experiment_dict['bag_robot_topics']}"
        if experiment_dict.get("bag_robot_topics") else ""
    )
    manager = LaneWindow(
        "manager", "gazebo test",
        f"ros2 launch gazebo_test experiment_manager.launch.py use_recorder:={args.bag_record}\
                use_evaluator:={args.hunav_eval} record_maps:={args.record_maps} \
                goals_and_poses_file:={goals_and_poses} \
                experiment_name:={experiment_identifier} \
                {base_path_for_results} \
                {algorithm_name} \
                {exp_config_pkg} \
                {wait_before_start} \
                {repetitions} \
                {checkpoint_arg} \
                {resume_checkpoint} \
                {navigation_backend_arg} \
                {watchdog_required_nodes_arg} \
                {params_file_arg} \
                {timeout_arg} \
                {stall_arg} \
                {episodes_arg} \
                {bag_topics_arg} \
                task:={task} ",
    )
    return LaneSpec(
        domain=ros_domain_id,
        gazebo_uri=gazebo_uri,
        env={"ROS_DOMAIN_ID": str(ros_domain_id), "GAZEBO_MASTER_URI": gazebo_uri},
        windows=windows,
        manager=manager,
        experiment=experiment_identifier,
        algorithm=selected_algorithm_name,
        params_file=navigator_path,
        notes=notes,
    )
