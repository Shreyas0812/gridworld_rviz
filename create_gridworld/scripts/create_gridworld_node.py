#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class CreateGridWorldNode(Node):
    def __init__(self):
        super().__init__("create_grid_world_node")
        self.get_logger().info("Hello Python")

def main(args=None):
    rclpy.init(args=args)
    node = CreateGridWorldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()