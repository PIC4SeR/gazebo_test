import os
from pathlib import Path
from typing import Dict
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, PoseStamped
from tf_transformations import quaternion_from_euler
import yaml


def parse_entity_state_yaml(yaml_path: Path) -> Dict[str, Dict[str, EntityState]]:
    """
    Parse data to create a list of EntityState objects.
    This method reads the yaml file from the provided path and converts it into
    a list of EntityState objects.
    The data should contain the necessary information to create EntityState
    objects, such as name, pose, and other attributes.


    Args:
        yaml_path (Path): Path to the YAML file containing the entity state data.
    Returns:
        Dict[str, Dict[str, EntityState]]: A dictionary with two keys:
            'initial_state_entities' and 'goal_entities', each containing a
            dictionary mapping episode names to their corresponding EntityState
            objects.
    """

    with open(yaml_path, "r") as file:
        data = yaml.safe_load(file)

    episodes = data.get("episodes", [])

    goals = data.get("goals", {})
    poses = data.get("poses", {})

    robot_name = data.get("robot_name", "robot")
    goal_name = data.get("goal_name", "goal_box")

    initial_state_entities = {}
    goal_entities = {}

    for episode in episodes:
        goal = goals.get(episode)
        pose = poses.get(episode)
        if goal is None or len(goal) < 2:
            raise ValueError(
                f"Episode '{episode}' is missing a valid 'goals' entry [x, y]."
            )
        if pose is None or len(pose) < 3:
            raise ValueError(
                f"Episode '{episode}' is missing a valid 'poses' entry [x, y, theta]."
            )

        goal_entity = EntityState()
        goal_entity.name = goal_name
        goal_entity.pose = Pose()
        goal_entity.pose.position.x = float(goal[0])
        goal_entity.pose.position.y = float(goal[1])

        initial_state_entity = EntityState()
        initial_state_entity.name = robot_name
        initial_state_entity.pose = Pose()

        # Position
        initial_state_entity.pose.position.x = float(pose[0])
        initial_state_entity.pose.position.y = float(pose[1])
        initial_state_entity.pose.position.z = 0.07
        # Orientation
        quaternion = quaternion_from_euler(0, 0, float(pose[2]))
        initial_state_entity.pose.orientation.x = quaternion[0]
        initial_state_entity.pose.orientation.y = quaternion[1]
        initial_state_entity.pose.orientation.z = quaternion[2]
        initial_state_entity.pose.orientation.w = quaternion[3]
        # Reference frame

        goal_entities[episode] = goal_entity
        initial_state_entities[episode] = initial_state_entity
    return {
        "initial_state_entities": initial_state_entities,
        "goal_entities": goal_entities,
    }


def _entity_from_pose(name: str, x: float, y: float, theta: float, z: float) -> EntityState:
    entity = EntityState()
    entity.name = name
    entity.pose = Pose()
    entity.pose.position.x = float(x)
    entity.pose.position.y = float(y)
    entity.pose.position.z = float(z)
    q = quaternion_from_euler(0, 0, float(theta))
    entity.pose.orientation.x = q[0]
    entity.pose.orientation.y = q[1]
    entity.pose.orientation.z = q[2]
    entity.pose.orientation.w = q[3]
    return entity


def parse_fleet_yaml(yaml_path: Path) -> dict:
    """Parse a multi-robot goals/poses YAML.

    Schema (the presence of a top-level ``robots:`` list selects multi-robot mode)::

        robots:
          - {name: jackal0, model: jackal,    spawn: [x, y, theta], nav2_params: <path>}
          - {name: tb3_1,   model: turtlebot3, spawn: [x, y, theta]}
        goal_name: goal_box            # per-robot goal box becomes "<goal_name>_<robot>"
        episodes: [episode_1, ...]
        goals:   {episode_1: {jackal0: [x, y],        tb3_1: [x, y]}}
        poses:   {episode_1: {jackal0: [x, y, theta], tb3_1: [x, y, theta]}}

    Returns a dict with keys ``fleet`` (list of robot dicts, namespace == name),
    ``initial_state_entities`` and ``goal_entities`` mapping
    episode -> {robot_name -> EntityState}.
    """
    with open(yaml_path, "r") as file:
        data = yaml.safe_load(file)

    fleet = data.get("robots")
    if not fleet:
        raise ValueError(
            f"'{yaml_path}' has no 'robots:' list; use parse_entity_state_yaml for "
            "single-robot configs."
        )
    robot_names = [r["name"] for r in fleet]
    goal_name = data.get("goal_name", "goal_box")
    episodes = data.get("episodes", [])
    goals = data.get("goals", {})
    poses = data.get("poses", {})

    initial_state_entities: Dict[str, Dict[str, EntityState]] = {}
    goal_entities: Dict[str, Dict[str, EntityState]] = {}
    for episode in episodes:
        ep_goals = goals.get(episode, {})
        ep_poses = poses.get(episode, {})
        initial_state_entities[episode] = {}
        goal_entities[episode] = {}
        for name in robot_names:
            goal = ep_goals.get(name)
            pose = ep_poses.get(name)
            if goal is None or len(goal) < 2:
                raise ValueError(
                    f"Episode '{episode}', robot '{name}' missing 'goals' [x, y]."
                )
            if pose is None or len(pose) < 3:
                raise ValueError(
                    f"Episode '{episode}', robot '{name}' missing 'poses' [x, y, theta]."
                )
            initial_state_entities[episode][name] = _entity_from_pose(
                name, pose[0], pose[1], pose[2], z=0.07
            )
            goal_entities[episode][name] = _entity_from_pose(
                f"{goal_name}_{name}", goal[0], goal[1], 0.0, z=0.0
            )
    return {
        "fleet": fleet,
        "initial_state_entities": initial_state_entities,
        "goal_entities": goal_entities,
    }


def get_posestamped_from_entity(entity: EntityState, frame_id: str) -> PoseStamped:
    """
    Get the PoseStamped object from the entity state.
    This method takes an EntityState object and returns a PoseStamped object
    with the pose of the entity.
    Args:
        entity (EntityState): The entity state object.
        frame_id (str): The frame ID for the pose stamped.
    Returns:
        PoseStamped: The pose stamped object with the goal pose.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose = entity.pose
    return pose_stamped
