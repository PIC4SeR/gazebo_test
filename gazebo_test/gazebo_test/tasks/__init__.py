"""Experiment tasks. Importing this package registers the built-in tasks."""

# Importing the modules runs their @register_task decorators, populating
# gazebo_test.tasks.base.TASKS. Add new built-in tasks here.
from gazebo_test.tasks import go_to_pose  # noqa: F401
from gazebo_test.tasks import go_to_pose_multirobot  # noqa: F401
