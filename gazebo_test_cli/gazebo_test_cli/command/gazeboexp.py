"""`ros2 gazeboexp` command extension.

Exposes the full_experiment orchestrator as a native ros2cli command so it can
be invoked as `ros2 gazeboexp run ...` / `ros2 gazeboexp list`, with tab
completion provided by ros2cli's own argcomplete integration (the argument
completers defined in full_experiment.build_parser carry over).

The standalone `full_experiment` console script remains available too.
"""

from ros2cli.command import CommandExtension

from gazebo_test_cli.full_experiment import build_parser, dispatch


class GazeboExpCommand(CommandExtension):
    """Run Gazebo social-navigation experiments."""

    def add_arguments(self, parser, cli_name):
        build_parser(parser)

    def main(self, *, parser, args):
        dispatch(args)
        return 0
