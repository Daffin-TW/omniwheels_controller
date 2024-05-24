#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from tf_transformations import euler_from_quaternion


class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        # Constructor
        


def main(args=None):
    rclpy.init(args=args)
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode has been stopped')
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()