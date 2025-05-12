# Installation Instructions

Follow the steps below to install and set up the `gazebo_test` package:

## Prerequisites

Ensure you have the following installed on your system:

- [ROS 2](https://docs.ros.org/en/rolling/Installation.html) (Humble or later)
- [Gazebo](https://gazebosim.org/) (Fortress or later)
- `colcon` build tool
- Python 3.8 or later

## Clone the Repository

```bash
cd ~/workspaces/hunavsim_ws/src
git clone https://github.com/your-repo/gazebo_test.git
```

## Install Dependencies

Navigate to the workspace root and install dependencies:

```bash
cd ~/workspaces/hunavsim_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build the Package

Build the workspace using `colcon`:

```bash
colcon build --packages-select gazebo_test
```

## Source the Workspace

Source the setup file to overlay the workspace:

```bash
source ~/workspaces/hunavsim_ws/install/setup.bash
```

## Run the Package

Launch the package with the following command:

```bash
ros2 launch gazebo_test <launch_file>.launch.py
```

Replace `<launch_file>` with the appropriate launch file for your use case.

## Troubleshooting

If you encounter issues, ensure all dependencies are installed and sourced correctly. Refer to the [ROS 2 documentation](https://docs.ros.org/) for additional help.
