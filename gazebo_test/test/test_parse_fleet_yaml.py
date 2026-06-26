"""Self-check for the multi-robot fleet parser.

Run directly (``python3 test_parse_fleet_yaml.py``) or under pytest. Needs a
sourced ROS environment for ``gazebo_msgs`` / ``tf_transformations``.
"""

import pathlib
import sys
import tempfile

# Allow running from the source tree before the package is installed.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from gazebo_test.utils.common_utils import parse_fleet_yaml  # noqa: E402

YAML = """
robots:
  - {name: jackal0, model: jackal,     spawn: [9.0, 9.0, -3.14], nav2_params: a.yaml}
  - {name: tb3_1,   model: turtlebot3, spawn: [8.0, 9.0, 0.0]}
goal_name: goal_box
episodes: [episode_1]
goals:
  episode_1: {jackal0: [2.49, 8.11], tb3_1: [4.0, 7.0]}
poses:
  episode_1: {jackal0: [8.8, 8.32, -3.0], tb3_1: [7.0, 8.0, 0.0]}
"""


def _write(text: str) -> pathlib.Path:
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    f.write(text)
    f.close()
    return pathlib.Path(f.name)


def test_parse_fleet_yaml():
    parsed = parse_fleet_yaml(_write(YAML))

    assert [r["name"] for r in parsed["fleet"]] == ["jackal0", "tb3_1"]

    inits = parsed["initial_state_entities"]["episode_1"]
    goals = parsed["goal_entities"]["episode_1"]
    assert set(inits) == {"jackal0", "tb3_1"}
    assert inits["jackal0"].name == "jackal0"
    assert abs(inits["jackal0"].pose.position.x - 8.8) < 1e-9
    assert abs(inits["jackal0"].pose.position.z - 0.07) < 1e-9  # spawned above ground
    # per-robot goal boxes get distinct names
    assert goals["jackal0"].name == "goal_box_jackal0"
    assert goals["tb3_1"].name == "goal_box_tb3_1"
    assert abs(goals["tb3_1"].pose.position.x - 4.0) < 1e-9


def test_missing_goal_raises():
    bad = YAML.replace("tb3_1: [4.0, 7.0]", "tb3_1: [4.0]")
    try:
        parse_fleet_yaml(_write(bad))
    except ValueError:
        return
    raise AssertionError("expected ValueError for incomplete goal")


# Formation config: no per-robot goals, a per-episode centroid target instead.
FORMATION_YAML = """
robots:
  - {name: jackal,  model: jackal, spawn: [9.0, 9.0, -3.14]}
  - {name: jackal1, model: jackal, spawn: [8.0, 9.0, -3.14]}
episodes: [episode_1]
poses:
  episode_1: {jackal: [8.8, 8.32, -3.0], jackal1: [7.8, 8.32, -3.0]}
centroid:
  episode_1: [2.49, 4.0]
"""


def test_formation_config_optional_goals_and_centroid():
    parsed = parse_fleet_yaml(_write(FORMATION_YAML))

    # Poses still parsed; goals omitted entirely (formation has none).
    assert set(parsed["initial_state_entities"]["episode_1"]) == {"jackal", "jackal1"}
    assert parsed["goal_entities"]["episode_1"] == {}
    # Per-episode swarm target centroid.
    assert parsed["centroid"]["episode_1"] == (2.49, 4.0)


def test_missing_pose_raises_even_without_goals():
    bad = FORMATION_YAML.replace("jackal1: [7.8, 8.32, -3.0]", "jackal1: [7.8, 8.32]")
    try:
        parse_fleet_yaml(_write(bad))
    except ValueError:
        return
    raise AssertionError("expected ValueError for incomplete pose")


# Single-robot config: the same schema, a 'robots:' list with one participant.
SINGLE_ROBOT_YAML = """
robots:
  - {name: jackal, model: jackal, spawn: [9.0, 9.0, -3.14]}
goal_name: goal_box
episodes: [episode_1]
goals:
  episode_1: {jackal: [2.41, 8.5]}
poses:
  episode_1: {jackal: [9.0, 8.18, -3.2]}
"""


def test_single_robot_fleet_of_one():
    parsed = parse_fleet_yaml(_write(SINGLE_ROBOT_YAML))

    assert [r["name"] for r in parsed["fleet"]] == ["jackal"]
    assert set(parsed["initial_state_entities"]["episode_1"]) == {"jackal"}
    assert parsed["goal_entities"]["episode_1"]["jackal"].name == "goal_box_jackal"
    assert abs(parsed["goal_entities"]["episode_1"]["jackal"].pose.position.x - 2.41) < 1e-9


def test_single_robot_config_rejected():
    try:
        parse_fleet_yaml(_write("robot_name: jackal\nepisodes: []\n"))
    except ValueError:
        return
    raise AssertionError("expected ValueError when 'robots:' is absent")


if __name__ == "__main__":
    test_parse_fleet_yaml()
    test_missing_goal_raises()
    test_formation_config_optional_goals_and_centroid()
    test_missing_pose_raises_even_without_goals()
    test_single_robot_fleet_of_one()
    test_single_robot_config_rejected()
    print("PASS parse_fleet_yaml self-check")
