# Gazebo Test

This repository contains utilities and scripts for testing Gazebo simulations in ROS2 in an automated way.
It is capable of running simulations, repeating them, and generating reports on the results.

The main goal is to provide a framework for testing Gazebo simulations in a consistent and repeatable manner.
The framework is designed to be extensible, allowing for the addition of new tests and features as needed.
The framework is designed to be used with ROS2 and Gazebo, but can be adapted for use with other simulation environments as well.

## Requirements

- ROS2 Humble
- Gazebo
- Python 3.10 or higher

## Installation

1. Clone the repository into your ROS2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/PIC4SeR/gazebo_test
   ```

2. Install the required dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

4. Source the workspace:

   ```bash
    source ~/ros2_ws/install/setup.bash
    ```

5. install the required python packages

   ```bash
   pip install --user -r src/gazebo_test/requirements.txt
   ```

6. Install tmux if not already installed

   ```bash
   sudo apt install tmux
   ```

7. Resolve the issue with tf_transformations

   change the lines in /usr/lib/python3/dist-packages/transforms3d/quaternions.py
   search for _MAX_FLOAT and _FLOAT_EPS and change them to:

   ```python
   _MAX_FLOAT = np.maximum_sctype(np.float32)
   _FLOAT_EPS = np.finfo(np.float32).eps
   ```

   This resolves the issue with tf_transformations in ROS2 Humble. (save the file as root to be able to edit it)

8. Add the following line to your ~/.tmux.conf file to enable mouse support in tmux:

   ```bash
   set -g mouse on
   ```

   This allows you to scroll and select panes using the mouse.

9. Change the base_path in gazebo_test/gazebo_experiments/experiment_config.yaml to your workspace path

   ```yaml
   base_path: "<your_workspace_path>/results"
   ```

10. Verify that you have the correct nav2_params files in gazebo_test/gazebo_experiments/nav2_params
    You can use the ones provided in the repository or create your own.

## Packages

- ['gazebo_test'](gazebo_test/README.md): The main package containing the test framework and utilities.
- ['gazebo_sim'](gazebo_sim/README.md): A package containing Gazebo simulation models and worlds for testing.

## Usage

Check the [usage documentation](docs/usage.md) for detailed instructions on how to use the framework.

## TODO

- rviz panel for experiment setting
