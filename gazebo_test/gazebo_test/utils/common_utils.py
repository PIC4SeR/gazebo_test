import os
from pathlib import Path
from typing import Dict
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, PoseStamped
from tf_transformations import quaternion_from_euler
import yaml


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
    """Parse a goals/poses YAML for any number of robots.

    This is the single parser for both the single-robot ``go_to_pose`` task and
    the multi-robot tasks: a single-robot config is just a ``robots:`` list with
    one participant. The presence of a top-level ``robots:`` list is required.

    Schema::

        robots:
          - {name: jackal0, model: jackal,    spawn: [x, y, theta], nav2_params: <path>}
          - {name: tb3_1,   model: turtlebot3, spawn: [x, y, theta]}
        goal_name: goal_box            # per-robot goal box becomes "<goal_name>_<robot>"
        episodes: [episode_1, ...]
        goals:   {episode_1: {jackal0: [x, y],        tb3_1: [x, y]}}   # optional
        poses:   {episode_1: {jackal0: [x, y, theta], tb3_1: [x, y, theta]}}
        centroid: {episode_1: [x, y]}                                 # optional

    ``poses`` are required. Per-robot ``goals`` are required only when present
    (the go-to-pose fleet task drives them); the formation task instead steers
    the whole swarm toward the per-episode ``centroid`` target, so it omits
    ``goals``.

    Returns a dict with keys ``fleet`` (list of robot dicts, namespace == name),
    ``initial_state_entities`` and ``goal_entities`` mapping
    episode -> {robot_name -> EntityState}, and ``centroid`` mapping
    episode -> (x, y) for episodes that declare one.
    """
    with open(yaml_path, "r") as file:
        data = yaml.safe_load(file)

    fleet = data.get("robots")
    if not fleet:
        raise ValueError(
            f"'{yaml_path}' has no 'robots:' list. Every goals/poses config -- "
            "single robot included -- must declare a 'robots:' list."
        )
    robot_names = [r["name"] for r in fleet]
    goal_name = data.get("goal_name", "goal_box")
    episodes = data.get("episodes", [])
    goals = data.get("goals", {})
    poses = data.get("poses", {})
    centroid_cfg = data.get("centroid", {})

    initial_state_entities: Dict[str, Dict[str, EntityState]] = {}
    goal_entities: Dict[str, Dict[str, EntityState]] = {}
    for episode in episodes:
        ep_goals = goals.get(episode, {})
        ep_poses = poses.get(episode, {})
        initial_state_entities[episode] = {}
        goal_entities[episode] = {}
        for name in robot_names:
            pose = ep_poses.get(name)
            if pose is None or len(pose) < 3:
                raise ValueError(
                    f"Episode '{episode}', robot '{name}' missing 'poses' [x, y, theta]."
                )
            initial_state_entities[episode][name] = _entity_from_pose(
                name, pose[0], pose[1], pose[2], z=0.07
            )
            # Per-robot goals are optional (formation has none); validate only
            # when a 'goals' block is present for the episode.
            if ep_goals:
                goal = ep_goals.get(name)
                if goal is None or len(goal) < 2:
                    raise ValueError(
                        f"Episode '{episode}', robot '{name}' missing 'goals' [x, y]."
                    )
                goal_entities[episode][name] = _entity_from_pose(
                    f"{goal_name}_{name}", goal[0], goal[1], 0.0, z=0.0
                )
    centroid = {
        episode: tuple(centroid_cfg[episode])
        for episode in episodes
        if episode in centroid_cfg
    }
    return {
        "fleet": fleet,
        "initial_state_entities": initial_state_entities,
        "goal_entities": goal_entities,
        "centroid": centroid,
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
