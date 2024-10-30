#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')

        # Create a subscriber to /scan with BEST_EFFORT QoS
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=best_effort_qos)

        # Create a publisher to /scan_relay with RELIABLE QoS
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.publisher = self.create_publisher(LaserScan, '/scan_relay', qos_profile=reliable_qos)

    def scan_callback(self, msg):
        # Republish the message on /scan_relay
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
