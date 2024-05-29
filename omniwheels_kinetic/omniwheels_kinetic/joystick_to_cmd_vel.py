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
        self.cmd_vel_ = (0.0, 0.0, 0.0)
        self.target_cmd_vel_ = (0.0, 0.0, 0.0)
        self.scaling_status_ = False
        self.stopped_status_ = True
        self.run_publish_status_ = False
        self.shooting_status_ = False
        self.request_status_ = False
        self.charging_status_ = False
        self.future_ = None
        self.request_ = Trigger.Request()
        self.turn_speed_ = 5
        self.acceleration_ = 0.05
        self.max_cmd_vel_ = 1
        
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
        self.timer_cmd_vel_ = self.create_timer(0.05, self.target_to_cmd_vel)
        self.timer_status_timer_ = self.create_timer(
            0.5, self.status_shooting_callback)
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

    # Controls Robot Acceleration and Velocity
    def target_to_cmd_vel(self):
        vel = [*self.cmd_vel_]

        for i in range(3):
            if round(self.target_cmd_vel_[i], 2) == round(self.cmd_vel_[i], 2):
                continue
            elif self.target_cmd_vel_[i] > self.cmd_vel_[i]:
                vel[i] = round(vel[i] + self.acceleration_, 3)
            else:
                vel[i] = round(vel[i] - self.acceleration_, 3)

        self.cmd_vel_ = tuple(vel)
        
    # cmd_vel Info
    def cmd_vel_info(self):
        max_v = self.max_cmd_vel_
        max_w = round(self.turn_speed_ / self.max_cmd_vel_, 3)
        cmd_vel_msg = '\n'.join([
            f'  vx: {self.cmd_vel_[0]}',
            f'  vy: {self.cmd_vel_[1]}',
            f'  vw: {self.cmd_vel_[2]}'
        ])
        # self.get_logger().info(msg)
        print('---', f'max linear speed: {max_v}', f'max angular speed {max_w}',
              'cmd_vel:', cmd_vel_msg, sep='\n')

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
                f'---\nService Respond:\n- Status\t: {self.response_.success}'
                '\n- Message\t: {self.response_.message}'
            )
        else:
            None

    # Subscriber Callback
    def callback_control(self, msg: Joy):
        joystick = {
            'LEFTX': msg.axes[0],
            'LEFTY': msg.axes[1],
            'RIGHTX': msg.axes[3],
            'LEFTTRIGGER': msg.axes[2],
            'RIGHTTRIGGER': msg.axes[5],
            'LEFTSHOULDER': msg.buttons[4],
            'RIGHTSHOULDER': msg.buttons[5],
            'X': msg.buttons[2],
            'A': msg.buttons[0]
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
        condition = (self.scaling_status_ !=
                     (joystick['LEFTSHOULDER'] or
                     joystick['RIGHTSHOULDER']))
        if condition:
            if self.scaling_status_:
                self.scaling_status_ = False
            else:
                self.scaling_status_ = True
                if joystick['LEFTSHOULDER'] and self.max_cmd_vel_ > 0.5:
                    self.max_cmd_vel_ -= 0.5
                    self.run_publish_status_ = True
                elif joystick['RIGHTSHOULDER'] and self.max_cmd_vel_ < 5:
                    self.max_cmd_vel_ += 0.5
                    self.run_publish_status_ = True

        # Converts Joystick's Inputs to Target Velocities
        if joystick['LEFTTRIGGER'] < 0.8 or joystick['RIGHTTRIGGER'] < 0.8:
            v1 = joystick['LEFTY'] * self.max_cmd_vel_
            v2 = joystick['LEFTX'] * self.max_cmd_vel_
            v3 = joystick['RIGHTX'] * self.turn_speed_ / self.max_cmd_vel_

            self.target_cmd_vel_ = (v1, v2, v3)
            self.stopped_status_ = False

        # If the Robot Stops
        elif not self.stopped_status_:
            self.target_cmd_vel_ = (0.0, 0.0, 0.0)
            self.run_publish_status_ = True
        
        if sum(abs(v) for v in self.cmd_vel_) == 0:
            self.stopped_status_ = True

    # Publish Charging State
    def publish_charging_state_(self):
        msg = Bool()
        msg.data = self.charging_status_
        self.pub_charge_.publish(msg)

    # Publish Robot Velocity
    def publish_messages(self):
        # self.debugging()
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

    def debugging(self):
        self.get_logger().info('Responding...')


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