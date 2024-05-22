#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from tf_transformations import euler_from_quaternion
from omniwheels_interfaces.msg import WheelsVelocity3


class WheelVelToCmdVel(Node):
    def __init__(self):
        super().__init__('wheelVel_to_cmdVel')

        # Constructor
        



def main(args=None):
    rclpy.init(args=args)
    node = WheelVelToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode has been stopped')
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
