#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from math import sqrt, atan2
import numpy as np
import sys

class ProportionalController(Node):
    def __init__(self):
        super().__init__('proportional_controller')

        # Robot Parameters
        self.wheelbase = .508 # value in meters, equivalent to 20 in
        # Publisher for controlling wheel velocity
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Publisher for controlling wheel angle
        self.wheel_angle_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        # Subscribe to the 'odom' topic to get the robot's position
        self.subscription = self.create_subscription(PoseStamped, 'odom', self.odom_callback, 10)

        # Target y position and control parameters
        self.target_x = 10.0  # Set desired x-coordinate
        self.target_y = 10.0  # Set desired y-coordinate 
        
        self.kp_linear = 1  # Proportional gain for linear velocity
        self.reached_goal = False

        self.get_logger().info('Straight line controller initialized. Moving towards x = 10.0 y = 10.0.')

        # Data logging arrays
        self.x_position = np.array([])
        self.y_position = np.array([])
        self.counter_ = 0.0


    def odom_callback(self, msg):
        """Callback to process PoseStamped messages and control the robot."""

        steer_angle = atan2(self.wheelbase, self.target_x) + .0073

        # Update the robot’s current y-position
        current_y = msg.pose.position.y
        current_x = msg.pose.position.x

        
        # Calculate distance to the goal

        # Due to slippage that occurs in the Gazebo simulation, the vehicle does not travel in a perfect arc. 
        # For this reason, it is not possible to determine accurate position based on current arc length vs desired point arc length.
        # Instead, distance from goal is determined solely based on y positon becuase it was found to yield accurate results.

        distance_to_goal = abs(self.target_y - current_y)

        self.get_logger().info(f"Wheel Angle: {steer_angle:.5f}, Current Position - x: {current_x:.2f}, Current Position - y: {current_y:.2f}, Distance to Goal: {distance_to_goal:.2f}")

        # Stop the robot if close enough to the goal
        if distance_to_goal < 0.2 and not self.reached_goal:
            self.stop_robot()
            self.reached_goal = True
            return
        if self.reached_goal:
            return

        # Proportional control for forward velocity based on distance to the goal
        linear_vel = self.kp_linear * distance_to_goal
    

        # Publish forward velocity command with opposite signs for wheels
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [linear_vel, -linear_vel]

        # Calculate and Publish wheel steering command with opposite signs for wheels
        wheel_angle = Float64MultiArray()
        wheel_angle.data = [steer_angle, -steer_angle]

        self.wheel_angle_pub.publish(wheel_angle)
        self.wheel_velocities_pub.publish(wheel_velocities)
        
        # Display published linear velocity in terminal
        self.get_logger().info(f"Publishing: linear_vel = {linear_vel}")
        self.get_logger().info(f"Publishing: linear_vel = {wheel_angle}")

        # Data logging for robot pose plots
        self.x_position = np.append(self.x_position, current_x)
        self.y_position = np.append(self.y_position, current_y)

        np.save('x_position', self.x_position)
        np.save('y_position', self.y_position)

    # Stops robot once it has reached its desired point
    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [0.0, 0.0]

        self.wheel_velocities_pub.publish(wheel_velocities)
        self.get_logger().info('Reached the goal! Stopping the robot.')

def main(args=None):
    rclpy.init(args=args)
    proportional_controller = ProportionalController()
    rclpy.spin(proportional_controller)
    proportional_controller.stop_robot()  # Stop the robot upon shutdown
    proportional_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
