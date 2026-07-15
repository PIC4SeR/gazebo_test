"""Orchestrate a full Gazebo social-navigation experiment in tmux.

The script brings up the Gazebo simulation, the Nav2 stack, RViz, an optional
HuNav evaluator, and the experiment manager, each in its own tmux window.

Shared by the standalone ``full_experiment`` console script (:func:`main`) and
the ros2cli command extension (``ros2 gazeboexp``).
"""

import argparse
import textwrap
import yaml
import libtmux
from libtmux.exc import LibTmuxException
import os
import time
import argcomplete
from argcomplete.completers import FilesCompleter
import shlex
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from typing import Any, Optional, List, Dict, Tuple
import warnings

from gazebo_test.script_utils.parsing_utils import (
    print_summary,
    parse_experiment_name,
    get_experiment_ids,
)

from gazebo_test.script_utils.navigator_utils import (
    get_navigator_additional_launch_files,
    get_navigator_navigation_launch,
    get_navigator_yaml,
    list_available_navigators,
)

from gazebo_test.utils.checkpoint_store import ExperimentCheckpointStore


def _print_experiments():
    """
    Print the list of experiments.
    """
    experiment_names = get_experiment_ids()
    print("Available experiments:\n")
    for name in experiment_names:
        print(f"      - {name}")


def _print_navigators():
    """
    Print the list of navigators.
    """
    navigator_names = list_available_navigators()
    print("Available navigators:\n")
    for name in navigator_names:
        print(f"      - {name}")


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


DESCRIPTION = textwrap.dedent(
    """\
    Run a full experiment in Gazebo with navigation and evaluation.
    Use the run subcommand to start the experiment.
    """
)


def build_parser(parser: argparse.ArgumentParser) -> argparse.ArgumentParser:
    """Populate ``parser`` with the run/list sub-commands and their completers.

    Shared by the standalone console script (:func:`main`) and the ros2cli
    command extension (``ros2 gazeboexp``).
    """
    sub_parser = parser.add_subparsers(dest="cmd", required=True)

    run_parser = sub_parser.add_parser(
        "run", help="Run a full experiment in Gazebo with navigation and evaluation."
    )
    run_parser.add_argument(
        "--bag-record", action="store_true", help="Enable bag recording"
    )
    run_parser.add_argument(
        "--hunav-eval",
        action="store_true",
        help="Enable hunav evaluation mode (use hunav_eval.launch.py)",
    )

    run_parser.add_argument(
        "--record-maps",
        action="store_true",
        help="Record map topics in the bag files (map, global and local costmap)",
    )

    run_parser.add_argument(
        "--headless", action="store_true", help="Run Gazebo in headless mode"
    )

    run_parser.add_argument("--no-rviz", action="store_true", help="Do not start RViz")

    run_parser.add_argument(
        "--wait-after-finish",
        action="store_true",
        help="Wait for the user to press Ctrl+C after the test is finished",
    )

    run_parser.add_argument(
        "--no-gpu",
        action="store_true",
        help="Do not use GPU for rendering (use CPU instead)",
    )

    experiment_action = run_parser.add_argument(
        "experiment",
        action="store",
        default="social_nav",
        help="The Experiment to run (chosen by the list of available experiments)\
            Hint: Use 'list' command to see all available experiments.",
    )
    experiment_action.completer = lambda **kwargs: get_experiment_ids()  # type: ignore[attr-defined]

    run_parser.add_argument(
        "--wait-before-start",
        type=int,
        default=None,
        help="Number of seconds to wait before starting the experiment (to let the environment settle)",
    )

    run_parser.add_argument(
        "--repetitions",
        type=int,
        default=None,
        help="Override the experiment configuration's repetition count",
    )

    nav_params_action = run_parser.add_argument(
        "--nav-params",
        action="store",
        default=None,
        help="The path to the navigation parameters file",
    )

    navigator_action = run_parser.add_argument(
        "--navigator",
        action="store",
        default=None,
        help="The navigation controller to use. \
        Hint: Use 'list' command to see all available controllers.",
    )
    navigator_action.completer = lambda **kwargs: list_available_navigators()  # type: ignore[attr-defined]

    run_parser.add_argument(
        "--checkpoint-dsn",
        action="store",
        default=os.environ.get("GAZEBO_TEST_CHECKPOINT_DSN", ""),
        help="PostgreSQL DSN for the shared checkpoint store.",
    )
    run_parser.add_argument(
        "--disable-checkpoint",
        action="store_true",
        help="Disable checkpoint/database usage even if a DSN is configured.",
    )

    run_parser.add_argument(
        "--resume-checkpoint",
        action="store_true",
        help="Resume experiments by skipping runs listed in the checkpoint file.",
    )

    eval_metrics_action = run_parser.add_argument(
        "--evaluation-metrics-file",
        action="store",
        default=os.path.join(
            get_package_share_directory("hunav_evaluator"),
            "config",
            "metrics.yaml",
        ),
        help="The path to the evaluation metrics file",
    )

    run_parser.add_argument(
        "--base-path-for-results",
        action="store",
        default=None,
        help="The base path for the results of the experiment",
    )
    run_parser.add_argument(
        "--algorithm-name",
        action="store",
        default=None,
        help="The name of the algorithm used in the experiment",
    )

    run_parser.add_argument(
        "--ros-domain-id",
        type=int,
        default=0,
        help="The ROS domain ID to isolate the experiment (0-1023)",
    )

    run_parser.add_argument(
        "--exp-config-pkg",
        action="store",
        default=None,
        help="The package containing the experiment configuration files",
    )

    run_parser.add_argument(
        "--additional-launch-files",
        nargs="*",
        default=[],
        action="extend",
        help=textwrap.dedent(
            "Additional launch files to run in the experiment. "
            "These are appended after additional_launch_files declared in the selected experiment and navigator YAML. "
            "You can specify multiple launch files by separating them with spaces. "
            "It is recommended to use quotes for each launch file if they contain spaces, "
            'e.g., --additional-launch-files "my_pkg my_launch.py" "another_pkg another_launch.py". '
            "Or you can use a single string if it is an absolute path, "
            'e.g., --additional-launch-files "/path/to/my_pkg my_launch.py".'
        ),
    )
    run_parser.add_argument(
        "--navigation-launch",
        action="store",
        default=None,
        help=textwrap.dedent(
            "Launch file to run in the Navigation tmux window instead of the default "
            "gazebo_test bringup.launch.py. Use the same syntax as additional launch "
            'files, e.g. --navigation-launch "my_nav_pkg my_nav.launch.py". '
            "Placeholders are supported: {map}, {params_file}, {nav_params}, "
            "{goals_and_poses}, {experiment}, {algorithm_name}, {ros_domain_id}, {gazebo_uri}."
        ),
    )
    run_parser.add_argument(
        "--no-navigation-launch",
        action="store_true",
        help="Do not start a navigation launch; use a navigation stack that is already running.",
    )
    run_parser.add_argument(
        "--navigation-backend",
        choices=["nav2", "action", "navigate_to_pose", "nav2_action"],
        default=None,
        help=(
            "Experiment manager navigation interface. Defaults to nav2 for the built-in "
            "launch and action when a custom or external launch is selected."
        ),
    )
    run_parser.add_argument(
        "--watchdog-required-nodes",
        action="store",
        default=None,
        help=(
            "Comma-separated node names required by the experiment watchdog. "
            "Use 'none' to disable node checks."
        ),
    )
    run_parser.add_argument(
        "--task",
        action="store",
        default=None,
        help=(
            "Experiment task (go_to_pose | multirobot | ...). Overrides the "
            "experiment's declared 'task'; defaults to go_to_pose."
        ),
    )

    # YAML/params path arguments complete to *.yaml files.
    _yaml_completer = FilesCompleter(("yaml", "yml"), directories=True)
    nav_params_action.completer = _yaml_completer  # type: ignore[attr-defined]
    eval_metrics_action.completer = _yaml_completer  # type: ignore[attr-defined]

    run_parser.set_defaults(func=run)
    ls_parser = sub_parser.add_parser("list", help="List all available experiments.")
    ls_parser.set_defaults(
        func=lambda args: [
            _print_experiments(),  # new line
            print(""),  # new line
            _print_navigators(),
        ]
    )  # new line
    return parser


def dispatch(args: argparse.Namespace) -> None:
    """Invoke the sub-command selected on ``args``."""
    args.func(args)


def main():
    parser = argparse.ArgumentParser(description=DESCRIPTION)
    build_parser(parser)
    argcomplete.autocomplete(parser, always_complete_options=False)
    dispatch(parser.parse_args())


def run(args: argparse.Namespace):
    """Run the specified experiment."""
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
        selected_algorithm_name = "unknown"
    # Must match the identifier the experiment manager registers jobs under
    # (ExperimentManager: experiment_name_param or goals-file stem). Experiments
    # that share a goals_and_poses file (e.g. *_crowded_env all use
    # crowded_env.yaml) would otherwise collapse to the same stem here and let a
    # completed sibling run skip this experiment's launch.
    experiment_identifier = args.experiment or Path(goals_and_poses).stem

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
                    return
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

    # Create a new tmux server
    server = libtmux.Server()

    # Check if the session already exists
    session_name = "gazebo_test_" + ros_domain_id.__str__()
    try:
        if session_name in server.sessions:
            print(f"Session '{session_name}' already exists. Exiting.")
            return
    except LibTmuxException:
        print("Tmux server not running. Starting a new server.")

    # Create a new tmux session

    with server.new_session(
        session_name=session_name,
        environment={
            "ROS_DOMAIN_ID": str(ros_domain_id),
            "GAZEBO_MASTER_URI": gazebo_uri,
        },
    ) as session:
        print(f"Created new session: {session_name}")
        print(
            f"Set ROS_DOMAIN_ID to {session.getenv('ROS_DOMAIN_ID')} and GAZEBO_MASTER_URI to {session.getenv('GAZEBO_MASTER_URI')}"
        )

        # Create a window for gazebo simulation
        window = session.active_window
        window.rename_window("Gazebo")
        print("Created new window: Gazebo")
        # Start the Gazebo simulation
        pane = window.panes[0]
        pane.select()
        gpu_env_var = (
            "__GLX_VENDOR_LIBRARY_NAME=nvidia __NV_PRIME_RENDER_OFFLOAD=1 "
            if not args.no_gpu
            else ""
        )
        robots_sim_arg = shlex.quote(f"robots:={robots_arg}") if robots_arg else ""
        robot_name_arg = f"robot_name:={primary_robot}" if primary_robot else ""
        pane.send_keys(
            f"{gpu_env_var}ros2 launch gazebo_sim simulation.launch.py headless:={args.headless} world_pkg_name:={world_pkg_name} base_world:={world} \
                agents_configuration_file:={agents_configuration_file} config_pkg_name:={agents_pkg_name} \
                    use_lidar_gpu:={not args.no_gpu} {robot_name_arg} {robots_sim_arg}"
        )
        print("Started Gazebo simulation")

        if navigation_launch_command:
            session.new_window(attach=True, window_name="Navigation")
            window = session.active_window
            window.rename_window("Navigation")
            print("Created new window: Navigation")
            # Start the navigation stack
            pane = window.panes[0]
            pane.select()
            pane.send_keys(f"ros2 launch {navigation_launch_command}")
            print(f"Started navigation launch: {navigation_launch_command}")
        else:
            print("Navigation launch disabled; expecting an external navigation stack.")

        # Create a new pane for RViz
        if not args.no_rviz:
            session.new_window(attach=True, window_name="RViz")
            window = session.active_window
            window.rename_window("RViz")
            print("Created new window: RViz")
            # Start RViz
            pane = window.panes[0]
            pane.select()
            # Lone robot: the stack runs namespaced (see
            # _default_navigation_launch_command), so RViz must follow it.
            rviz_robots_arg = (
                shlex.quote(f"robots:={robots_arg}")
                if robots_arg
                else f"namespace:={primary_robot}"
            )
            pane.send_keys(f"ros2 launch gazebo_test rviz.launch.py {rviz_robots_arg}")
            print("Started RViz")
        if args.hunav_eval:
            session.new_window(attach=True, window_name="Hunav Eval")
            window = session.active_window
            window.rename_window("Hunav Eval")
            print("Created new window: Hunav Eval")
            # Start the hunav evaluation
            pane = window.panes[0]
            pane.select()
            pane.send_keys(
                f"ros2 launch hunav_evaluator hunav_evaluator_launch.py \
                metrics_file:={args.evaluation_metrics_file}"
            )

            print("Started hunav evaluation")

        # For each additional launch file, create a new pane and run the launch file.
        for launch_file in additional_launch_files:
            window_name = _launch_window_name(launch_file)
            session.new_window(attach=True, window_name=window_name)
            window = session.active_window
            window.rename_window(window_name)
            print(f"Created new window: {window_name}")
            # Start the additional launch file
            pane = window.panes[0]
            pane.select()
            pane.send_keys(f"ros2 launch {launch_file}")
            print(f"Started additional launch file: {launch_file}")

        session.new_window(attach=True)
        window = session.active_window
        window.rename_window("gazebo test")
        print("Created new window: gazebo test")
        # Start the gazebo test
        pane = window.panes[0]
        pane.select()

        base_path_for_results = (
            f"base_path:={args.base_path_for_results}"
            if args.base_path_for_results
            else ""
        )
        algorithm_name = ""
        if not args.algorithm_name:
            if args.navigator:
                algorithm_name = f"algorithm_name:={args.navigator}"
        else:
            algorithm_name = f"algorithm_name:={args.algorithm_name}"

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
        if checkpoint_enabled and checkpoint_dsn_value:
            checkpoint_arg = f"checkpoint_dsn:={checkpoint_dsn_value}"
            print("Using PostgreSQL checkpoint store via provided DSN")
        elif not checkpoint_enabled and checkpoint_dsn_value:
            # Should not happen because disabled mode clears the DSN, but keep for safety.
            checkpoint_dsn_value = ""

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
        pane.send_keys(
            f"ros2 launch gazebo_test experiment_manager.launch.py use_recorder:={args.bag_record}\
                use_evaluator:={args.hunav_eval} record_maps:={args.record_maps} \
                goals_and_poses_file:={goals_and_poses} \
                experiment_name:={args.experiment} \
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
                task:={task} \
                ; tmux wait-for -S process_finished_{ros_domain_id}"
        )
        print("Started gazebo test")

        # run until the user presses Ctrl+C or until the gazebo test is finished

        try:

            server.cmd("wait-for", f"process_finished_{ros_domain_id}")

            print("Gazebo test finished")
            # retrieve the log of the gazebo test
            current_panel = "".join(pane.capture_pane())
            # get the path of the log file
            log_path: Optional[str] = None
            for line in current_panel.splitlines():
                if "Experiment outcomes saved to" in line:
                    log_path = line.split("Experiment outcomes saved to", 1)[1]
                    log_path = log_path.split("[")[0].strip()
                    break
            if log_path:
                print_summary(Path(log_path))
            else:
                print(
                    "Unable to locate the experiment outcomes file in the log output; summary not generated."
                )
            # wait until the keyboard interrupt
            if args.wait_after_finish:
                print("Press Ctrl+C to exit...")
                while True:
                    time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            # Cleanup: kill the session
            print("Exiting...")


if __name__ == "__main__":
    main()
