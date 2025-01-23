#!/usr/bin/python3

import rclpy
from rclpy.node import Node


class PointToPointControllerNode(Node):
    def __init__(self):
        super().__init__('point_to_point_controller_node')

def main(args=None):
    rclpy.init(args=args)
    node = PointToPointControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
