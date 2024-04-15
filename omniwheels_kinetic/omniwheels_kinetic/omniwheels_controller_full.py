#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped
from omniwheels_interfaces.msg import WheelsVelocity3

from math import cos, sin, radians


class Omniwheels_Velocity3(Node):
    def __init__(self):
        super().__init__('omniwheels_vel')

        # Constructor
        self.cmd_vel_ = (float(), float(), float())
        self.wheels_vel_ = (float(), float(), float())
        
        # Topics
        self.subcriber_ = self.create_subscription(
            Joy, 'joy', self.callback_control, 10)
        self.pub_wheels_ = self.create_publisher(
            WheelsVelocity3, 'wheels_vel', 10)
        self.pub_cmd_vel_ = self.create_publisher(
            TwistStamped, 'cmd_vel', 10)
        
        # Start Info
        self.get_logger().info('Node has been started')


    # Normalize Robot Velocity
    def vector_normalization(self, x: float, y: float, omega: float) -> tuple:
        length = ((x**2) + (y**2) + ((omega**2)*2))**0.5

        if length > 1:
            return (x/length, y/length, omega/(length*2))
        elif length:
            return (x, y, omega)
        else:
            return (0.0, 0.0, 0.0)


    # Wheels Velocity Calculation
    def wheels_movement(self, x: float, y: float, omega: float) -> tuple[float]:
        v, vn, w = self.vector_normalization(x, y, omega)

        L = 1
        gama = 30
        v1 = (-vn + w*L)
        v2 = (v*cos(radians(gama))) + (vn*sin(radians(gama))) + (w*L)
        v3 = (-v*cos(radians(gama))) + (vn*sin(radians(gama))) + (w*L)

        return (v1, v2, v3)


    # Wheels Velocity Info
    def wheels_velocity_info(self):
        msg = '\n'.join(f'Wheel {i+1}: {self.wheels_vel_[i]}' for i in range(3))
        # self.get_logger().info(msg)
        print('---', 'Translation:', msg, sep='\n')


    # Converts Joystick Inputs to Robot Velocity
    def joy_to_cmd_vel(self, joystick: dict[str, float | int], scale: float) -> tuple[float]:
        v1 = joystick['LEFTX'] * scale
        v2 = joystick['LEFTY'] * scale
        v3 = joystick['RIGHTX'] * scale
        return (v1, v2, v3)


    # Subscriber Callback
    def callback_control(self, msg: Joy) -> None:
        joystick = {
            'LEFTX': msg.axes[0],
            'LEFTY': msg.axes[1],
            'RIGHTX': msg.axes[3],
            'LEFTSHOULDER': msg.buttons[4],
            'RIGHTSHOULDER': msg.buttons[5]
        }

        # Determine The Robot Velocity Scaling
        if joystick['RIGHTSHOULDER']:
            scale = 1.0
        elif joystick['LEFTSHOULDER']:
            scale = 0.7
        else:
            # Else Not Publishing Anything
            return None

        self.cmd_vel_ = self.joy_to_cmd_vel(joystick, scale)
        self.wheels_vel_ = self.wheels_movement(*self.cmd_vel_)

        self.publish_cmd_vel()
        self.publish_wheels_vel()
        self.wheels_velocity_info()


    # Publish Robot Velocity
    def publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = self.cmd_vel_[0]
        twist.linear.y = self.cmd_vel_[1]
        twist.angular.z = self.cmd_vel_[2]

        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist = twist

        self.pub_cmd_vel_.publish(msg)


    # Publish Wheels Velocity
    def publish_wheels_vel(self):
        msg = WheelsVelocity3()
        msg.wheel1 = self.wheels_vel_[0]
        msg.wheel2 = self.wheels_vel_[1]
        msg.wheel3 = self.wheels_vel_[2]
        self.pub_wheels_.publish(msg)



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