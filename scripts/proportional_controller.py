#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from math import sqrt, atan2, pi

class TurtleBot3Controller(Node):
    def __init__(self):
        super().__init__('turtlebot3_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(PoseStamped, 'odom', self.odom_callback, 10)
        self.target_x = 10.0  # Set your desired target coordinates here
        self.target_y = 2.0
        self.kp_linear = 0.9  # Linear velocity gain
        self.kp_angular = 0.8  # Angular velocity gain
        self.position = {'x': 0.0, 'y': 0.0}
        self.orientation = 0.0  # Robot's current yaw
        self.reached_goal = False  # Goal reached flag
        
        self.get_logger().info('TurtleBot3 go-to-goal controller initialized.')

    def quaternion_to_yaw(self, orientation_q):
        """Convert quaternion to yaw (Z-axis rotation)."""
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        return atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Callback function to handle odometry messages."""
        # Get the robot's current position
        self.position['x'] = msg.pose.position.x
        self.position['y'] = msg.pose.position.y
        # Get the robot's current yaw (orientation)
        self.orientation = self.quaternion_to_yaw(msg.pose.orientation)
        
        error_x = self.target_x - self.position['x']
        error_y = self.target_y - self.position['y']
        distance_to_goal = sqrt(error_x ** 2 + error_y ** 2)

        # Stop the robot if it's close enough to the goal
        if distance_to_goal < 0.1 and not self.reached_goal:
            self.stop_robot()
            self.reached_goal = True
            return

        if self.reached_goal:
            return

        # Proportional control for linear and angular velocities
        angle_to_goal = atan2(error_y, error_x)
        angle_error = angle_to_goal - self.orientation

        # Normalize the angle to the range [-pi, pi]
        angle_error = (angle_error + pi) % (2 * pi) - pi

        vel_msg = Twist()
        vel_msg.linear.x = max(self.kp_linear * distance_to_goal, 0.1)
        vel_msg.angular.z = self.kp_angular * angle_error

        self.get_logger().info(f'Velocity command - Linear: {vel_msg.linear.x:.2f}, Angular: {vel_msg.angular.z:.2f}')
        self.get_logger().info(f'Position: x={self.position["x"]}, y={self.position["y"]}; Distance to goal: {distance_to_goal:.2f}')

        self.publisher_.publish(vel_msg)

    def stop_robot(self):
        """Publish zero velocity to stop the robot."""
        vel_msg = Twist()
        self.publisher_.publish(vel_msg)
        self.get_logger().info('Reached the goal! Stopping the robot.')

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_controller = TurtleBot3Controller()
    rclpy.spin(turtlebot3_controller)
    turtlebot3_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
