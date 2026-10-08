from pathlib import Path

import yaml


def test_all_swarm_profile_combinations_are_registered():
    path = (
        Path(__file__).parents[2]
        / "gazebo_experiments"
        / "experiments"
        / "experiments.yaml"
    )
    config = yaml.safe_load(path.read_text())
    for potential in ("gaussian", "moussaid", "combined", "none"):
        for memory in ("off", "raw", "normalized"):
            name = f"perturbation_tb2_{potential}_{memory}"
            assert name in config["experiments"]
            launch = config[name]["navigation_launch"]
            assert launch["package"] == "swarm_control"
            assert launch["arguments"]["profile"] == potential
            assert launch["arguments"]["memory_profile"] == memory


def test_mixed_fleet_formation_profile_matrix():
    package = Path(__file__).parents[2] / "gazebo_experiments"
    config = yaml.safe_load(
        (package / "experiments" / "swarm_formation_profiles.yaml").read_text()
    )
    for scenario in ("formation_passing_tb2", "formation_crossing_tb2"):
        goals = yaml.safe_load(
            (package / "goals_and_poses" / f"{scenario}.yaml").read_text()
        )
        assert [robot["model"] for robot in goals["robots"]] == [
            "jackal",
            "turtlebot2",
            "turtlebot2",
        ]
        assert goals["robots"][0]["max_speed"] == 0.35
        assert goals["robots"][0]["max_lin_accel"] == 0.3
        for episode in goals["episodes"]:
            assert set(goals["poses"][episode]) == {
                "jackal",
                "turtlebot2",
                "turtlebot2_1",
            }
        for potential in ("gaussian", "moussaid", "combined", "none"):
            for memory in ("off", "raw", "normalized"):
                name = f"{scenario}_{potential}_{memory}"
                assert name in config["experiments"]
                arguments = config[name]["navigation_launch"]["arguments"]
                assert arguments["profile"] == potential
                assert arguments["memory_profile"] == memory
                assert arguments["odom_topic"] == "ground_truth"


def test_formation_esogeno_tb2_is_registered_and_faithful_to_the_reference():
    """The MATLAB moussaid_esogeno scene, wired like the other tb2 formations."""
    package = Path(__file__).parents[2] / "gazebo_experiments"
    config = yaml.safe_load(
        (package / "experiments" / "swarm_formation_profiles.yaml").read_text()
    )
    goals = yaml.safe_load(
        (package / "goals_and_poses" / "formation_esogeno_tb2.yaml").read_text()
    )
    # Four TurtleBot2, not the mixed jackal+2 trio: the reference's control law
    # is per-robot identical, so a mixed fleet would confound the comparison.
    assert [robot["model"] for robot in goals["robots"]] == ["turtlebot2"] * 4
    for episode in goals["episodes"]:
        assert len(goals["poses"][episode]) == 4

    for name in (
        "formation_esogeno_tb2_esogeno",
        "formation_esogeno_tb2_gaussian_off",
        "formation_esogeno_tb2_moussaid_off",
        "formation_esogeno_tb2_combined_off",
        "formation_esogeno_tb2_none_off",
    ):
        assert name in config["experiments"]
        arguments = config[name]["navigation_launch"]["arguments"]
        assert arguments["odom_topic"] == "ground_truth"
        assert config[name]["agents_configuration_file"] == "agents_envs/esogeno/agents.yaml"

    # The reference variant must carry all three of the things that make it the
    # reference: the profile, the versor memory, and the EXOGENOUS centroid law.
    reference = config["formation_esogeno_tb2_esogeno"]["navigation_launch"]["arguments"]
    assert reference["profile"] == "moussaid_esogeno"
    assert reference["memory_profile"] == "normalized"
    assert reference["centroid_mode"] == "exogenous"


def test_esogeno_agents_use_the_reference_social_force_gains():
    """compute_human_forces.m is lightsfm; these three gains are the only ones
    HuNav exposes, and the rest are lightsfm defaults that already match."""
    agents = yaml.safe_load(
        (Path(__file__).parents[2] / "gazebo_sim" / "config" / "agents_envs"
         / "esogeno" / "agents.yaml").read_text()
    )["hunav_loader"]["ros__parameters"]
    # cv would route around the force computation entirely.
    assert agents["default_motion_model"] == "sfm"
    for name in agents["agents"]:
        behavior = agents[name]["behavior"]
        assert behavior["goal_force_factor"] == 2.0        # omega_g
        assert behavior["obstacle_force_factor"] == 10.0   # omega_o
        assert behavior["social_force_factor"] == 2.1      # omega_s, not HuNav's 5.0
        assert behavior["type"] == 1                       # REGULAR: reacts to robots


def test_esogeno_exogenous_input_is_derived_so_the_run_can_terminate():
    """An EXPLICIT u_ex drives until cancelled -- in a walled room, into a wall.

    controller.exogenous_input only zeroes on arrival when CentroidDriver
    DERIVED u_ex from the goal; an explicit one bypasses that entirely. On an
    unbounded plane (the MATLAB reference) that is correct. Here it meant the
    fleet sailed past its y=+3.5 goal into the north wall at y=+9.36 --
    captured as corridor::Wall_15::Wall_15_Collision, reported as "Collision
    with environment". The "0.0,0.0" sentinel is what makes it derived.

    It must also override the moussaid_esogeno PROFILE's own u_ex: [1.0, 0.0],
    which is explicit and points along the reference's axis, not the room's.
    """
    package = Path(__file__).parents[2] / "gazebo_experiments"
    config = yaml.safe_load(
        (package / "experiments" / "swarm_formation_profiles.yaml").read_text()
    )
    arguments = config["formation_esogeno_tb2_esogeno"]["navigation_launch"]["arguments"]
    u_ex = [float(v) for v in arguments["u_ex"].split(",")]
    assert u_ex == [0.0, 0.0], "explicit u_ex never stops; it must be derived"
    # And the derived magnitude must be executable by a TurtleBot2.
    assert 0.0 < float(arguments["uc_speed"]) <= 0.35


def test_esogeno_profile_u_ex_is_explicit_and_therefore_room_unsafe():
    """Pins WHY the experiment overrides it, so the override is not 'tidied' away."""
    profile = yaml.safe_load(
        (Path(__file__).parents[3] / "swarm_control" / "config" / "profiles"
         / "moussaid_esogeno.yaml").read_text()
    )["/**"]["ros__parameters"]
    assert profile["u_ex"] == [1.0, 0.0]
    assert profile["centroid_mode"] == "exogenous"


def test_esogeno_tracked_cell_uses_the_closed_loop_goal_tracker():
    """The open-vs-closed-loop pair must differ ONLY in the centroid law.

    Same scene, profile and memory as formation_esogeno_tb2_esogeno; the whole
    point is that a difference in the result is attributable to the centroid law
    and nothing else. 'goal' takes no u_ex -- it has no constant input to
    configure -- so its presence here would be a copy-paste error.
    """
    package = Path(__file__).parents[2] / "gazebo_experiments"
    config = yaml.safe_load(
        (package / "experiments" / "swarm_formation_profiles.yaml").read_text()
    )
    assert "formation_esogeno_tb2_tracked" in config["experiments"]
    tracked = config["formation_esogeno_tb2_tracked"]
    esogeno = config["formation_esogeno_tb2_esogeno"]

    # Identical scene: same world, map, agents, poses.
    for key in ("goals_and_poses", "map", "world", "agents_configuration_file"):
        assert tracked[key] == esogeno[key]

    a, b = tracked["navigation_launch"]["arguments"], esogeno["navigation_launch"]["arguments"]
    assert a["profile"] == b["profile"] == "moussaid_esogeno"
    assert a["memory_profile"] == b["memory_profile"] == "normalized"
    assert a["odom_topic"] == "ground_truth"
    assert a["centroid_mode"] == "goal" and b["centroid_mode"] == "exogenous"
    assert "u_ex" not in a, "'goal' has no constant input; u_ex here is a copy-paste"
    assert 0.0 < float(a["uc_speed"]) <= 0.35


def test_esogeno_sweep_cells_differ_from_the_control_only_in_social_and_memory():
    """36 cells; each must be its scenario's control with ONE of profile/memory
    swapped, so a Gazebo difference is attributable to the swept factor. The CA
    axis rides on --nav-params, hence params_file must stay the placeholder."""
    package = Path(__file__).parents[2] / "gazebo_experiments"
    config = yaml.safe_load(
        (package / "experiments" / "swarm_formation_profiles.yaml").read_text()
    )
    socials = {"none": "esogeno_none", "gaussian": "esogeno_gaussian",
               "moussaid": "moussaid_esogeno", "combined": "esogeno_combined"}
    for scenario in ("formation_esogeno_tb2", "formation_crossing_tb2",
                     "formation_passing_far_tb2"):
        control = config[f"{scenario}_tracked"]
        control_args = control["navigation_launch"]["arguments"]
        for social, profile in socials.items():
            for memory in ("off", "raw", "normalized"):
                name = f"{scenario}_eso_{social}_{memory}"
                assert name in config["experiments"], name
                cell = config[name]
                for key in ("goals_and_poses", "map", "world",
                            "agents_configuration_file"):
                    assert cell[key] == control[key], (name, key)
                args = cell["navigation_launch"]["arguments"]
                # `off` must arrive as the STRING "off", not YAML's false.
                assert args["memory_profile"] == memory, name
                assert args["profile"] == profile, name
                assert args["params_file"] == "{nav_params}", name
                for key in (set(args) | set(control_args)) - {"profile",
                                                               "memory_profile"}:
                    assert args.get(key) == control_args.get(key), (name, key)
