from gazebo_test_cli.launch_utils import swarm_algorithm_name


def test_swarm_algorithm_name_uses_profiles():
    command = (
        "swarm_control hybrid.launch.py profile:=moussaid "
        "memory_profile:=raw robots:='[{name: jackal}]'"
    )
    assert swarm_algorithm_name(command) == "moussaid_raw"


def test_swarm_algorithm_name_uses_launch_defaults():
    assert (
        swarm_algorithm_name("swarm_control hybrid.launch.py")
        == "combined_normalized"
    )


def test_swarm_algorithm_name_ignores_other_launches():
    assert swarm_algorithm_name("nav2_bringup navigation_launch.py") is None
