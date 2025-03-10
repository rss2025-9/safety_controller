#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")

        self.stop_thresh = 0.4  # meters before stopping
        self.stop_speed = 0.0  # stopping speed
        # self.max_decel = 2.5 # estimated maximum car deceleration in m/s 
        # self.min_speed = 1.0 # minimum speed for stopping - this is when we call publish stop command 
        # self.braking_speed = 0.5 # braking speed - how much to slow down gradually 
        self.current_speed = 0.0 # current speed updated based on drive msgs 
        self.current_steer = 0.0 # current angle based off drive msgs

        # Declare ROS params
        self.declare_parameter("use_real_racecar", True)  #for real car vs. sim
        self.declare_parameter("drive_topic", "/drive")  #default for simulation
        self.declare_parameter("safety_topic", "/vesc/low_level/input/safety")  #for real car

        # Get params
        self.use_real_racecar = self.get_parameter("use_real_racecar").get_parameter_value().bool_value
        self.DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter("safety_topic").get_parameter_value().string_value

        # Output topic based on real car or sims 
        self.output_topic = self.SAFETY_TOPIC if self.use_real_racecar else self.DRIVE_TOPIC

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.output_topic, 1)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1)
        self.ackermann_sub = self.create_subscription(AckermannDriveStamped, "/vesc/high_level/input/nav_0", self.ackermann_callback, 10)

    def scan_callback(self, msg):
        # LIDAR to np array
        ranges = np.array(msg.ranges)
        angles=np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # only -pi/6 to pi/6
        # mask = (angles >= self.current_steer-np.pi/6) & (angles <= self.current_steer+np.pi/6)
        mask = (angles >= -np.pi/6) & (angles <= np.pi/6)
        good_range=ranges[mask]

        # bad data out
        good_range = good_range[(good_range>msg.range_min) & (good_range<msg.range_max)]

        # closest thing to hit
        if len(good_range) > 0:
            closest_pt = np.min(good_range)
        else:
            closest_pt = float('inf')  #no obstacle

        # self.get_logger().info(f"Closest point: {closest_pt:.3f}m")  #to debug

        # if closer than threshold, stop
        # stopping_distance = max(self.stop_thresh, self.current_speed**2 / (2 * self.max_decel))
        if closest_pt < self.stop_thresh:
            # self.brake()
            # self.get_logger().warn("Braking")
            self.get_logger().warn("Publishing stop command")
            self.get_logger().info(f"Closest point: {closest_pt:.3f}m")  #to debug

            self.publish_stop_command()

    def ackermann_callback(self, msg):
        # intercepts driving command
        # self.get_logger().info(f"Received Drive Command: Speed={msg.drive.speed}, Steering={msg.drive.steering_angle}")
        self.current_speed = msg.drive.speed
        self.current_steer = msg.drive.steering_angle
    
    def brake(self): 
        stop_msg = AckermannDriveStamped()
        if self.current_speed > self.min_speed: 
            new_speed = max(self.current_speed - self.braking_speed, self.min_speed)
            stop_msg.drive.speed = float(new_speed)
            stop_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(stop_msg)
            self.get_logger().info(f"Braking: New speed = {new_speed} m/s")
        else: 
            self.publish_stop_command()

    def publish_stop_command(self):
        # pub stop command
        stop_msg=AckermannDriveStamped()
        stop_msg.drive.speed=self.stop_speed
        stop_msg.drive.steering_angle=0.0
        self.drive_pub.publish(stop_msg)
        self.get_logger().info("Stopping") #for debugging

def main():
    rclpy.init()
    safety_controller=SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
