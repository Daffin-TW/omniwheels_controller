#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class SequentialNode(Node):
    def __init__(self):
        super().__init__('sequential_node')

        # Constructor
        self.cmd_vel_ = (0.0, 0.0, 0.0)
        self.pressing_status_ = False
        self.move_n_meters_ = 0.0

        # Topics
        self.sub_joy_ = self.create_subscription(
            Joy, 'joy', self.callback_control, 10)
        self.pub_cmd_vel_ = self.create_publisher(
            Twist, 'cmd_vel', 10)
        
        # Timer
        self.timer_publish_ = self.create_timer(0.08, self.publish_messages)
        self.create_timer(0.01, self.move_n_meters)

        # Start Info
        self.get_logger().info('Node has been started')

    # Move n Meters
    def move_n_meters(self):
        if self.move_n_meters_ > 0:
            self.cmd_vel_ = (1.0, 0.0, 0.0)
            self.move_n_meters_ -= 0.01
        else:
            self.cmd_vel_ = (0.0, 0.0, 0.0)
    
    # cmd_vel Info
    def cmd_vel_info(self):
        cmd_vel_msg = '\n'.join([
            f'  vx: {self.cmd_vel_[0]}',
            f'  vy: {self.cmd_vel_[1]}',
            f'  vw: {self.cmd_vel_[2]}'
        ])
        # self.get_logger().info(msg)
        print('cmd_vel:', cmd_vel_msg, sep='\n')

    # Subscriber Callback
    def callback_control(self, msg: Joy):
        joystick = {
            'X': msg.buttons[2],
            'A': msg.buttons[0]
        }

        # Trigger Button to Move n Meters
        if joystick['A'] != self.pressing_status_:
            if self.pressing_status_:
                self.pressing_status_ = False
            else:
                self.pressing_status_ = True
                self.move_n_meters_ += 1

    # Publish Messages
    def publish_messages(self):
        msg = Twist()
        msg.linear.x = self.cmd_vel_[0]
        msg.linear.y = self.cmd_vel_[1]
        msg.angular.z = self.cmd_vel_[2]

        self.run_publish_status_ = False

        self.cmd_vel_info()
        self.pub_cmd_vel_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SequentialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode has been stopped')
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()