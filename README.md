## PROJECT 1 - Teleop and Proportional Controller

This README provides instructions for executing the package, along with commands for teleoperation and running a proportional controller. This package allows users to control a custom robot model in Gazebo using ROS 2 Galactic, either through manual teleoperation or an automated proportional controller.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#Installation)
- [Teleoperation](#teleoperation)
- [Proportional Controller](#proportional-controller)
- [Visualizing LiDAR Data and Robot Model in RViz](#Visualizing LiDAR Data and Robot Model in RViz)


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

## Visualizing LiDAR Data and Robot Model in RViz

To visualize the robot model and Lidar data, follow these steps:

1. **Launch RViz**: Open a new terminal and run the following command to start RViz with the appropriate configuration:

        ros2 launch project_1_group_2 display.launch.py

2. **Run the Laser Scan Node**: Open another terminal and execute the laser scan script to publish the LiDAR data:

        ros2 run project_1_group_2 laser_scan.py

3. **Configure RViz**:

  - Add the "RobotModel" display type.Set the topic to /robot_description to visualize the robot model.
  - In RViz, add the "LaserScan" display type. Set the topic to /scan_relay to visualize the incoming LiDAR points.
  - Add the "TF" display type. Ensure that the base link of the robot is displayed by setting it up with the relevant frame (e.g., base_link).
