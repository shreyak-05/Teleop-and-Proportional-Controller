#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from math import sqrt


# This node was oringally modified from the Turtlebot3 position controller node. 
# We used this as a starting point to understand proportional controllers, and it serves as the
# foundation for our actual proportional controller. The difference between this controller and the final
# controller, is that this proportional controller makes the robot travel only in a straight line.


class StraightLineController(Node):
    def __init__(self):
        super().__init__('straight_line_controller')

        # Publisher for controlling wheel velocity
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Subscribe to the 'odom' topic to get the robot's position
        self.subscription = self.create_subscription(PoseStamped, 'odom', self.odom_callback, 10)

        # Target y position and control parameters
        self.target_y = -5.0  # Set desired y-coordinate to move toward
        self.kp_linear = 1  # Proportional gain for linear velocity
        self.reached_goal = False

        self.get_logger().info('Straight line controller initialized. Moving towards y = 5.0.')

    def odom_callback(self, msg):
        """Callback to process PoseStamped messages and control the robot."""
        # Update the robot’s current y-position
        current_y = msg.pose.position.y

        # Calculate distance to the goal
        distance_to_goal = abs(self.target_y - current_y)
        self.get_logger().info(f"Current Position - y: {current_y:.2f}, Distance to Goal: {distance_to_goal:.2f}")

        # Stop the robot if close enough to the goal
        if distance_to_goal < 0.1 and not self.reached_goal:
            self.stop_robot()
            self.reached_goal = True
            return
        if self.reached_goal:
            return

        # Proportional control for forward velocity based on distance to the goal
        linear_vel = self.kp_linear * distance_to_goal

        # Cap the velocity to prevent overshooting near the goal
        # linear_vel = min(linear_vel, 0.2)

        # Publish forward velocity command with opposite signs for wheels
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [linear_vel, -linear_vel]

        self.wheel_velocities_pub.publish(wheel_velocities)
        self.get_logger().info(f"Publishing: linear_vel = {linear_vel}")

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [0.0, 0.0]

        self.wheel_velocities_pub.publish(wheel_velocities)
        self.get_logger().info('Reached the goal! Stopping the robot.')

def main(args=None):
    rclpy.init(args=args)
    straight_line_controller = StraightLineController()
    rclpy.spin(straight_line_controller)
    straight_line_controller.stop_robot()  # Stop the robot upon shutdown
    straight_line_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()