##PROJECT 1 - Teleop and Proportional Controller

This README provides instructions for executing the package, along with commands for teleoperation and running a proportional controller. This package allows users to control a custom robot model in Gazebo using ROS 2 Galactic, either through manual teleoperation or an automated proportional controller.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Teleoperation](#teleoperation)
- [Proportional Controller](#proportional-controller)


## Prerequisites

Ensure the following are installed and configured:

- **Ubuntu 20.04** with **ROS 2 Galactic**
- **Gazebo** simulator
- **Workspace Setup**: The package should be in a ROS 2 workspace, built, and sourced.

   ```bash
   colcon build
   source install/setup.bash

# Teleoperation

To run the robot with the proportional controller in competition world, use:

bash

    ros2 launch project_1_group_2 competition.launch.py


Once the robot is launched in Gazebo, follow these steps for teleoperation:

**Run Teleoperation**

In a separate terminal, start the teleoperation script to control the robot:

    bash

    ros2 run project_1_group_2 teleop.py

# Proportional Controller

Launch the ROS 2 environment and run the proportional controller:

bash

    ros2 run project_1_group_2 proportional_controller.py

