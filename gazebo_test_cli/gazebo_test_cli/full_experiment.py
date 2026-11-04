"""Orchestrate a full Gazebo social-navigation experiment in tmux.

The script brings up the Gazebo simulation, the Nav2 stack, RViz, an optional
HuNav evaluator, and the experiment manager, each in its own tmux window. What
runs is computed by :func:`gazebo_test_cli.lane_spec.build_lane_spec`; this
module only puts it in tmux (:func:`run_tmux`).

Shared by the standalone ``full_experiment`` console script (:func:`main`) and
the ros2cli command extension (``ros2 gazeboexp``).
"""

import argparse
import textwrap
import libtmux
from libtmux.exc import LibTmuxException
import os
import time
import argcomplete
from argcomplete.completers import FilesCompleter
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from typing import Optional

from gazebo_test.script_utils.parsing_utils import (
    print_summary,
    get_experiment_ids,
)

from gazebo_test.script_utils.navigator_utils import (
    list_available_navigators,
)

from gazebo_test_cli.lane_spec import LaneSpec, build_lane_spec


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
    run_parser.add_argument(
        "--episodes",
        action="store",
        default=None,
        help="Comma-separated subset of the goals file's episodes to run "
        "(e.g. episode_1,episode_3); default all.",
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
    """Run the specified experiment in a tmux session."""
    spec = build_lane_spec(args)
    if spec is None:
        return
    run_tmux(spec, wait_after_finish=args.wait_after_finish)


def run_tmux(spec: LaneSpec, wait_after_finish: bool = False) -> None:
    """Run a lane as the tmux session ``gazebo_test_<domain>``: one window per
    process, the experiment manager last; returns when the manager ends."""
    ros_domain_id = spec.domain
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

    with server.new_session(session_name=session_name, environment=spec.env) as session:
        print(f"Created new session: {session_name}")
        print(
            f"Set ROS_DOMAIN_ID to {session.getenv('ROS_DOMAIN_ID')} and GAZEBO_MASTER_URI to {session.getenv('GAZEBO_MASTER_URI')}"
        )
        for index, lane_window in enumerate(spec.windows):
            if index == 0:
                # The session's first window hosts the first process (Gazebo).
                window = session.active_window
            else:
                session.new_window(attach=True, window_name=lane_window.name)
                window = session.active_window
            window.rename_window(lane_window.name)
            print(f"Created new window: {lane_window.name}")
            pane = window.panes[0]
            pane.select()
            pane.send_keys(lane_window.command)
            print(f"Started {lane_window.name}: {lane_window.command}")
        if spec.window("nav") is None:
            print("Navigation launch disabled; expecting an external navigation stack.")

        session.new_window(attach=True)
        window = session.active_window
        window.rename_window(spec.manager.name)
        print(f"Created new window: {spec.manager.name}")
        pane = window.panes[0]
        pane.select()
        for note in spec.notes:
            print(note)
        # The tmux wait-for signal marks the end of the batch (the 16 spaces
        # keep the command byte-identical to the pre-LaneSpec CLI).
        pane.send_keys(
            spec.manager.command + " " * 16
            + f"; tmux wait-for -S process_finished_{ros_domain_id}"
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
            if wait_after_finish:
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
