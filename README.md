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

2. **Build and source the Workspace**: Compile the package by building the workspace and source the setup file to overlay this workspace onto your environment:

   ```bash
   colcon build
   source install/setup.bash

## Teleoperation

To launch the robot in competition world, use:

    ros2 launch project_1_group_2 competition.launch.py

In a separate terminal, start the teleoperation script to control the robot:

    ros2 run project_1_group_2 teleop.py

## Proportional Controller

To launch the robot in empty world, use: 

    ros2 launch project_1_group_2 gazebo.launch.py

In a separate terminal, run the proportional controller:

    ros2 run project_1_group_2 proportional_controller.py
    
The robot will begin moving from (0,0) to (10,10) using the proportional control strategy.
