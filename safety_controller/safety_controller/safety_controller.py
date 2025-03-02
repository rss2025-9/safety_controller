#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np

# Helper functions 
def angle_to_index(angle, scan):
    # get an index of scan from a desired angle 
    clamped_angle = max(scan.angle_min, min(angle, scan.angle_max))
    idx = int(round((clamped_angle - scan.angle_min) / scan.angle_increment))
    idx = max(0, min(idx, len(scan.ranges) - 1))
    return idx

def polar_to_cartesian(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")

        # ROS Parameters Declaration 
        self.declare_parameter('safety_topic', '/vesc/low_level/input/safety')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/vesc/low_level/ackermann_cmd')
        self.declare_parameter('safety_threshold', 0.25) # meters

        # Get Parameters 
        self.safety_topic = self.get_parameter('safety_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.drive_topic = self.get_parameter('drive topic').value
        self.safety_threshold = self.get_parameter('safety_threshold').value

        # Subscribers 
        self.scan_subscriber = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        # Publishers 
        self.safety_publisher = self.create_publisher(AckermannDriveStamped, self.safety_topic, 10)

    def scan_callback(self, scan): 
        mid_start = angle_to_index(-math.pi/15, scan)
        mid_end = angle_to_index(math.pi/15, scan)
        mid_ranges = np.array(scan.ranges[mid_start: mid_end])
        valid_mid_ranges = np.isfinite(mid_ranges) 
        min_distance = min(valid_mid_ranges) 
        if min_distance < self.safety_threshold: 
            self.publish_stop()

    def publish_stop(self): 
        stop = AckermannDriveStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.drive.speed = 0.0 
        stop.drive.steering_angle = 0.0
        self.safety_publisher.publish(stop)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()