#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoystickToVelocity(Node):
    def __init__(self):
        super().__init__('omniwheels_vel')

        # Constructor
        self.cmd_vel_ = (float(), float(), float())
        self.stopped_status_ = True
        self.run_publish_status_ = False
        self.shooting_status_ = False
        self.request_status_ = False
        self.charging_status_ = False
        self.future_ = None
        self.request_ = Trigger.Request()
        
        # Topics and Client Service
        self.subcriber_ = self.create_subscription(
            Joy, 'joy', self.callback_control, 10)
        self.pub_cmd_vel_ = self.create_publisher(
            Twist, 'cmd_vel', 10)
        self.pub_charge_ = self.create_publisher(
            Bool, 'charging', 10
        )
        self.client_ = self.create_client(Trigger, 'ball_shoot')

        # Timer
        self.timer_status_timer_ = self.create_timer(0.5, self.status_shooting_callback)
        self.timer_publish_ = self.create_timer(0.08, self.publish_messages)
        
        # Start Info
        self.get_logger().info('Node has been started')

    # Normalize Robot Velocity
    def vector_normalization(self, x: float, y: float, omega: float) -> tuple:
        length = ((x**2) + (y**2) + (omega**2))**0.5

        if length > 1:
            return (x/length, y/length, omega/length)
        elif length:
            return (x, y, omega)
        else:
            return (0.0, 0.0, 0.0)

    # Converts Joystick Inputs to Robot Velocity
    def joy_to_cmd_vel(self, joystick: dict[str, float | int], scale: float) -> tuple[float]:
        v1 = joystick['LEFTY'] * scale
        v2 = joystick['LEFTX'] * scale
        v3 = joystick['RIGHTX'] * scale * 6
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
    def request_shooting(self):
        if self.client_.service_is_ready():
            if not self.request_status_:
                self.get_logger().info('Requesting robot to shoot')
                self.future_ = self.client_.call_async(self.request_)
                self.request_status_ = True
            else:
                self.get_logger().info('Still waiting a respond from service server')
        else:
            self.get_logger().info('Service is not available')

    # Getting Result From Service Server using Timer
    def status_shooting_callback(self):
        if self.request_status_ and self.future_.done():
            self.request_status_ = False
            self.response_ = self.future_.result()
            self.get_logger().info('Service server has responded')
            print(
                f'---\nService Respond:\n- Status\t: {self.response_.success}\n- Message\t: {self.response_.message}'
            )
        else:
            None

    # Subscriber Callback
    def callback_control(self, msg: Joy):
        joystick = {
            'LEFTX': -msg.axes[0],
            'LEFTY': -msg.axes[1],
            'RIGHTX': -msg.axes[3],
            'LEFTSHOULDER': msg.buttons[4],
            'RIGHTSHOULDER': msg.buttons[5],
            'X': msg.buttons[2],
            'A': msg.buttons[1]
        }
        
        # Trigger Button to Shoot The Ball
        if self.shooting_status_ != joystick['X']:
            if self.shooting_status_:
                self.shooting_status_ = False
            else:
                self.shooting_status_ = True
                self.request_shooting()

        # Trigger Button to Start Charging
        if self.charging_status_ != bool(joystick['A']):
            self.charging_status_ = bool(joystick['A'])
            
        # Determine The Robot Velocity Scaling
        if joystick['RIGHTSHOULDER']:
            scale = 1.0
            self.stopped_status_ = False

        elif joystick['LEFTSHOULDER']:
            scale = 0.7
            self.stopped_status_ = False

        elif not self.stopped_status_:
            self.cmd_vel_ = (0.0, 0.0, 0.0)
            self.stopped_status_ = True
            self.run_publish_status_ = True
            return None

        else:
            return None

        self.cmd_vel_ = self.joy_to_cmd_vel(joystick, scale)
        # self.cmd_vel_ = self.vector_normalization(*self.cmd_vel_)

    # Publish Charging State
    def publish_charging_state_(self):
        msg = Bool()
        msg.data = self.charging_status_
        self.pub_charge_.publish(msg)

    # Publish Robot Velocity
    def publish_messages(self):
        self.publish_charging_state_()

        if self.stopped_status_ and not self.run_publish_status_:
            return None

        msg = Twist()
        msg.linear.x = self.cmd_vel_[0]
        msg.linear.y = self.cmd_vel_[1]
        msg.angular.z = self.cmd_vel_[2]

        self.run_publish_status_ = False

        self.cmd_vel_info()
        self.pub_cmd_vel_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickToVelocity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode has been stopped')
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
