# Gazebo Test

This repository contains utilities and scripts for testing Gazebo simulations in ROS2 in an automated way.
It is capable of running simulations, repeating them, and generating reports on the results.

The main goal is to provide a framework for testing Gazebo simulations in a consistent and repeatable manner.
The framework is designed to be extensible, allowing for the addition of new tests and features as needed.
The framework is designed to be used with ROS2 and Gazebo, but can be adapted for use with other simulation environments as well.

## Requirements

- ROS2 Humble
- Gazebo
- Python 3.8 or higher

## Installation

1. Clone the repository into your ROS2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone 
   ```

2. Install the required dependencies:

   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

4. Source the workspace:

   ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Packages

- ['gazebo_test'](gazebo_test/README.md): The main package containing the test framework and utilities.
- ['gazebo_sim'](gazebo_sim/README.md): A package containing Gazebo simulation models and worlds for testing.

## Usage

Check the [usage documentation](docs/usage.md) for detailed instructions on how to use the framework.
