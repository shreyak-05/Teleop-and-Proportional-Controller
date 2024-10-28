#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
import sys


# Import message type
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Create a Node that reads cmd_vel data and logs it into a data files for viewing 
class MySubscribeNode(Node):
    def __init__(self):
        super().__init__("odom_subscriber")
    
        # Creates np.arrays for storing relevant time and velocity data
        self.position_data = np.array([])
        self.time_data = np.array([])
        
        # Initializing counter for timer 
        self.counter_ = 0.0

        # Subscribing to the cmd_vel topic
        self.subscription = self.create_subscription(Odometry,'/odom', self.callback, 10)


    # Callback for collecting and storing time and velocity data 
    def callback(self, msg):

        self.position_data = np.append(self.position_data, format(msg.x,".3g"))
        self.counter_ += 0.1
        self.time_data = np.append(self.time_data, format(self.counter_, ".3g"))
        
        # Data Logging to Terminal 
        #self.get_logger().info('Time_Data: ' + str(self.time_data) + ' position_data: ' + str(self.position_data))

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.get_logger().info(f'Position: x={position.x}, y={position.y}, z={position.z}')
        self.get_logger().info(f'Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}')

        np.save('position_data_s1', self.position_data)
        np.save('time_data_s1', self.time_data)

        
def main(args=None):
    print('Running Truck_and_Trailer_subsriber')
    rclpy.init(args=args)

    node = MySubscribeNode()


    rclpy.spin(node)


    print(node.time_data)
    rclpy.shutdown()


    

if __name__ == '__main__':
    main()