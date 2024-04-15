#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Omniwheels_Velocity3(Node):
    def __init__(self):
        super().__init__('omniwheels_vel')

        # Constructor
        self.cmd_vel_ = (float(), float(), float())
        self.shooting_status_ = 0
        self.request_ = Trigger.Request()
        
        # Topics and Client Service
        self.subcriber_ = self.create_subscription(
            Joy, 'joy', self.callback_control, 10)
        self.publisher_ = self.create_publisher(
            Twist, 'cmd_vel', 10)
        self.client_ = self.create_client(Trigger, 'ball_shoot')
        
        # Start Info
        self.get_logger().info('Node has been started')


    # Normalize Robot Velocity
    def vector_normalization(self, x: float, y: float, omega: float) -> tuple:
        length = ((x**2) + (y**2) + (omega**2))**0.5

        if length > 1:
            return (x/length, y/length, omega/(length))
        elif length:
            return (x, y, omega)
        else:
            return (0.0, 0.0, 0.0)


    # Converts Joystick Inputs to Robot Velocity
    def joy_to_cmd_vel(self, joystick: dict[str, float | int], scale: float) -> tuple[float]:
        v1 = joystick['LEFTX'] * scale
        v2 = joystick['LEFTY'] * scale
        v3 = joystick['RIGHTX'] * scale
        return (v1, v2, v3)
        

    # cmd_vel Info
    def cmd_vel_info(self):
        msg = '\n'.join([
            f'vx: {self.cmd_vel_[0]}',
            f'vy: {self.cmd_vel_[1]}',
            f'vw: {self.cmd_vel_[2]}'
        ])
        # self.get_logger().info(msg)
        print('---', 'cmd_vel:', msg, sep='\n')


    # Request The Robot to Shoot
    def request_shooting(self) -> Trigger.Response | None:
        if self.client_.service_is_ready():
            self.future_ = self.client_.call_async(self.request_)
            rclpy.spin_until_future_complete(self, self.future_)
            self.get_logger().info(
                    f'Message: {self.response_.message}, Status: {self.response_.success}'
                )
            return self.future_.result()
        else:
            self.get_logger().info('Service is not available')
            return None


    # Subscriber Callback
    def callback_control(self, msg: Joy) -> None:
        joystick = {
            'LEFTX': msg.axes[0],
            'LEFTY': msg.axes[1],
            'RIGHTX': msg.axes[3],
            'LEFTSHOULDER': msg.buttons[4],
            'RIGHTSHOULDER': msg.buttons[5],
            'X': msg.buttons[2]
        }
        
        # Decision to Shoot The Ball
        if self.shooting_status_ != joystick['X']:
            if self.shooting_status_:
                self.shooting_status_ = 0
            else:
                self.shooting_status_ = 1
                self.response_ = self.request_shooting()
            
        # Determine The Robot Velocity Scaling
        if joystick['RIGHTSHOULDER']:
            scale = 1.0
        elif joystick['LEFTSHOULDER']:
            scale = 0.7
        else:
            # Else Not Publishing Anything
            return None

        self.cmd_vel_ = self.joy_to_cmd_vel(joystick, scale)
        self.cmd_vel_ = self.vector_normalization(*self.cmd_vel_)

        self.publish_cmd_vel()
        self.cmd_vel_info()


    # Publish Robot Velocity
    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.cmd_vel_[0]
        msg.linear.y = self.cmd_vel_[1]
        msg.angular.z = self.cmd_vel_[2]

        self.publisher_.publish(msg)


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
