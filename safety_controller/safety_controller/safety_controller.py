#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class SafetyController(Node):
    def __init__(self):
        super().__init__("safety_controller")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()