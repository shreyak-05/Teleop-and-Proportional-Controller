## PROJECT 1 - Teleop and Proportional Controller

This README provides instructions for executing the package, along with commands for teleoperation and running a proportional controller. This package allows users to control a custom robot model in Gazebo using ROS 2 Galactic, either through manual teleoperation or an automated proportional controller.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#Installation)
- [Teleoperation](#teleoperation)
- [Proportional Controller](#proportional-controller)


## Prerequisites

Ensure the following are installed and configured:

- **Ubuntu 20.04** with **ROS 2 Galactic**
- **Gazebo** simulator
- **Workspace Setup**: The package should be in a ROS 2 workspace, built, and sourced.

## Installation

To build the package from GitHub, follow these steps:

1. **Clone the Repository**: Clone the project repository into the `src` directory of your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/GraysonGilbert/project_1_group_2.git

2. **Build the Workspace**: Compile the package by building the workspace:

   ```bash
   colcon build

3. **Source the Workspace**: Source the setup file to overlay this workspace onto your environment:
   
   source install/setup.bash

## Teleoperation

To launch the robot in competition world, use:

    ros2 launch project_1_group_2 competition.launch.py


Once the robot is launched in Gazebo, follow these steps for teleoperation:

**Run Teleoperation**

In a separate terminal, start the teleoperation script to control the robot:

    ros2 run project_1_group_2 teleop.py

## Proportional Controller

Launch the ROS 2 environment and run the proportional controller:

    ros2 run project_1_group_2 proportional_controller.py

