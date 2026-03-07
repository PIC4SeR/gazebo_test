# rviz_goal_pose_annotator

RViz custom panel for creating `goals_and_poses` YAML files.

## Features

- Listens to `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`) as start pose source.
- Listens to `/goal_pose` (`geometry_msgs/PoseStamped`) as goal pose source.
- Lets you capture a start pose and a goal pose from the most recent tool poses.
- Suggests the next episode name from the selected YAML file (`episode_1`, `episode_2`, ...).
- Lets you set `map_package` and `map_path`, stores both in output YAML.
- `map_server` is launched from the launch file.
- RViz map rendering is handled by the provided `.rviz` config (`map` display already configured there).
- Appends the episode to:
  - `robot_name`
  - `map_path`
  - `map_package`
  - `initial_pose`
  - `episodes`
  - `goals`
  - `poses`
  in the same style as `gazebo_experiments/goals_and_poses/social_env_test.yaml`.

## Build

```bash
colcon build --packages-select rviz_goal_pose_annotator
source install/setup.bash
```

## Launch

Run both RViz and map publishing from launch:

```bash
ros2 launch rviz_goal_pose_annotator annotator.launch.py
```

Optional args:

- `start_map_server:=false` to skip launching map_server.
- `map_yaml_path:=/abs/path/to/map.yaml` to point to another map.
- `rviz_config_file:=/abs/path/to/file.rviz`.
- `start_map_server` also triggers a `nav2_lifecycle_manager` lifecycle transition (`autostart: true`) for `map_server`.
- Select the map in the panel with **Choose map path** to call `/map_server/load_map` (service `nav2_msgs/srv/LoadMap`).

## Use

1. Start RViz.
2. Add panel: **Panels -> GoalPoseAnnotatorPanel**.
3. Use your Gazebo/RViz tools:
   - `2D Pose Estimate` should publish `/initialpose` (start).
   - `2D Goal Pose` should publish `/goal_pose` (goal).
4. Click **Capture start from latest /initialpose** and **Capture goal from latest /goal_pose**.
5. Optionally edit episode name.
6. Click **Append episode to YAML**.
