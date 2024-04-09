#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from omniwheels_interfaces.msg import WheelsVelocity3

from math import cos, sin, radians


class Omniwheels_Velocity3(Node):
    def __init__(self):
        super().__init__('omniwheels_vel')

        self.robot_vel_ = (float(), float(), float())
        self.wheels_vel_ = (float(), float(), float())
        
        self.subcriber_ = self.create_subscription(
            Twist, 'omniwheel/robot_vel', self.callback_robot_vel, 10)
        self.publishers_ = self.create_publisher(
            WheelsVelocity3, 'omniwheel/wheels_vel', 10)
        
        self.get_logger().info('Node has been started')


    def scale_list(self, value: list | tuple) -> tuple:
        raise NotImplemented('Fungsi skala nilai-nilai dari elemen')


    def wheels_movement(self, x: float, y: float, theta: float) -> tuple[float]:
        L = 1
        gama = 30
        
        v1 = -y + theta*L
        v2 = x*cos(radians(gama)) + y*sin(radians(gama)) + theta*L
        v3 = -x*cos(radians(gama)) + y*sin(radians(gama)) + theta*L
        
        # Not Implemented Yet
        # return self.scale_list((v1, v2, v3))
    
        return (v1, v2, v3)


    def wheels_velocity_info(self):
        msg = '\n'.join(f'Wheel {i+1}: {self.wheels_vel_[i]}' for i in range(3))
        # self.get_logger().info(msg)
        print('---', 'Translation:', msg, sep='\n')


    def callback_robot_vel(self, msg: Twist):
        self.robot_vel_ = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.wheels_vel_ = self.wheels_movement(*self.robot_vel_)
        self.wheels_velocity_info()
        self.publish_wheels_vel()


    def publish_wheels_vel(self):
        msg = WheelsVelocity3()
        msg.wheel1 = self.wheels_vel_[0]
        msg.wheel2 = self.wheels_vel_[1]
        msg.wheel3 = self.wheels_vel_[2]
        self.publishers_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Omniwheels_Velocity3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode has been stopped')
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()