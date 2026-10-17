from gazebo_test_cli.launch_utils import swarm_algorithm_name


def test_swarm_algorithm_name_uses_the_profile():
    command = "swarm_control hybrid.launch.py profile:=adaptive n_robots:=4"
    assert swarm_algorithm_name(command) == "adaptive"


def test_swarm_algorithm_name_uses_the_launch_default():
    assert swarm_algorithm_name("swarm_control hybrid.launch.py") == "adaptive"


def test_swarm_algorithm_name_ignores_other_launches():
    assert swarm_algorithm_name("nav2_bringup navigation_launch.py") is None
