#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from math import sqrt, atan2, pi

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')

        # Publishers for controlling joint position and wheel velocity
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Subscribe to the 'odom' topic, which is of type PoseStamped
        self.subscription = self.create_subscription(PoseStamped, 'odom', self.odom_callback, 10)

        # Target location and control parameters
        self.target_x = -1.0
        self.target_y = -1.0
        self.kp_linear = 1
        self.kp_angular = 1
        self.position = {'x': 0.0, 'y': 0.0}  # Initial position set to (0, 0)
        self.orientation = 0.0
        self.reached_goal = False

        self.get_logger().info('Proportional go-to-goal controller initialized with PoseStamped odom.')

    def quaternion_to_yaw(self, orientation_q):
        """Convert quaternion to yaw (Z-axis rotation)."""
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        return atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Callback to process PoseStamped messages for position and orientation."""
        # Update the robot’s position and orientation using PoseStamped
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y
        self.orientation = self.quaternion_to_yaw(msg.pose.orientation)

        # Calculate errors in x, y, and distance to the goal
        error_x = self.target_x - self.position['x']
        error_y = self.target_y - self.position['y']
        distance_to_goal = sqrt(error_x ** 2 + error_y ** 2)

        # Print debugging information
        self.get_logger().info(f"Current Position - x: {self.position['x']:.2f}, y: {self.position['y']:.2f}")
        self.get_logger().info(f"Distance to Goal - x: {error_x:.2f}, y: {error_y:.2f}, Total: {distance_to_goal:.2f}")

        # Stop the robot if close enough to the goal
        if distance_to_goal < 0.1 and not self.reached_goal:
            self.stop_robot()
            self.reached_goal = True
            return
        if self.reached_goal:
            return

        # Calculate control actions
        angle_to_goal = atan2(error_y, error_x)
        angle_error = angle_to_goal - self.orientation
        angle_error = (angle_error + pi) % (2 * pi) - pi

        # Control linear and angular velocities
        linear_vel = max(self.kp_linear * distance_to_goal, 0.1)
        steer_angle = max(min(self.kp_angular * angle_error, 0.7), -0.7)

        # Publish control commands
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()

        wheel_velocities.data = [linear_vel, -linear_vel]
        joint_positions.data = [steer_angle, -steer_angle]

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)

    def stop_robot(self):
        """Stop the robot by publishing zero commands."""
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()
        wheel_velocities.data = [0.0, 0.0]
        joint_positions.data = [0.0, 0.0]

        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)
        self.get_logger().info('Reached the goal! Stopping the robot.')

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = TurtleBot3Controller()
    rclpy.spin(turtlebot3_controller)
    turtlebot3_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()