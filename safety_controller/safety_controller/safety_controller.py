#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


class safety_controller(Node):

    def __init__(self):
        super().__init__("safety_controller")

        self.stop_thresh=0.5  #meters before stopping
        self.stop_speed=0.0  #stopping speed

        #declare ROS params
        self.declare_parameter("use_real_racecar", False)  #for real car vs. sim
        self.declare_parameter("drive_topic", "/drive")  #default for simulation
        self.declare_parameter("safety_topic", "/vesc/low_level/input/safety")  #for real car

        #get params
        self.use_real_racecar=self.get_parameter("use_real_racecar").get_parameter_value().bool_value
        self.DRIVE_TOPIC=self.get_parameter("drive_topic").get_parameter_value().string_value
        self.SAFETY_TOPIC=self.get_parameter("safety_topic").get_parameter_value().string_value

        #based on environ
        self.output_topic=self.SAFETY_TOPIC if self.use_real_racecar else self.DRIVE_TOPIC

        #pub, as said in prompt
        self.drive_pub=self.create_publisher(AckermannDriveStamped, self.output_topic, 10)

        #sub, as said in prompt
        self.scan_sub=self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.ackermann_sub=self.create_subscription(AckermannDriveStamped, "/vesc/low_level/ackermann_cmd", self.ackermann_callback, 10)


    def scan_callback(self, msg):
        #LIDAR to detenct potential collisions

        #LIDAR to np array
        ranges=np.array(msg.ranges)
        angles=np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        #only -pi/2 to pi/2
        mask=(angles>=-np.pi/2)&(angles<=np.pi/2)
        good_range=ranges[mask]

        #bad data out
        good_range=good_range[(good_range>msg.range_min)&(good_range<msg.range_max)]

        #closest thing to hit
        if len(good_range)>0:
            closest_pt=np.min(good_range)
        else:
            closest_pt=float('inf')  #no obstacle

        self.get_logger().info(f"Closest point: {closest_pt:.3f}m")  #to debug

        #if closer than threshold, stop
        if closest_pt<self.stop_thresh:
            self.get_logger().warn("Publishing stop command")
            self.publish_stop_command()





    def ackermann_callback(self, msg):
        #intercepts driving commant
        self.get_logger().info(f"Received Drive Command: Speed={msg.drive.speed}, Steering={msg.drive.steering_angle}")

    def publish_stop_command(self):
       #pub stop command
        stop_msg=AckermannDriveStamped()
        stop_msg.drive.speed=self.stop_speed
        stop_msg.drive.steering_angle=0.0
        self.drive_pub.publish(stop_msg)

        self.get_logger().info("Stopping") #for debugging


def main():
    rclpy.init()
    safety_controller=safety_controller()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


