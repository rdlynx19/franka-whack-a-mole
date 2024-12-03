"""Integrate the hammer actuation with ROS"""
import rclpy
from rclpy.node import Node


class Swing(Node):
    def __init__(self):
        super().__init__('swing')

def main(args=None):
    rclpy.init(args=args)
    node = Swing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    import sys
    main(args=sys.argv)