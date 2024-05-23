#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from tf_transformations import euler_from_quaternion
from omniwheels_interfaces.msg import WheelsVelocity3, EncoderPulse


class WheelVelToCmdVel(Node):
    def __init__(self):
        super().__init__('wheelVel_to_cmdVel')

        # Constructor
        self.cmd_vel_ = TwistStamped()
        self.wheel_vel_ = WheelsVelocity3()
        
        # Topics
        self.subscriber_ = self.create_subscription(
            EncoderPulse, 'encoder_pulse', self.encoder_callback, 10)
        self.publisher_ = self.create_publisher(
            TwistStamped, 'cmd_vel_stamped', 10
        )

        # Timer
        self.pub_timer = self.create_timer(0.08, self.publish_messages)

    # Convert Regular Message to Ros Message
    def wheel_velocity3(self, Vm1: float, Vm2: float, 
                        Vm3: float) -> WheelsVelocity3:
        msg = WheelsVelocity3()

        msg.wheel1 = Vm1
        msg.wheel2 = Vm2
        msg.wheel3 = Vm3

        return msg
    
    def cmd_vel_twist(self, Vx: float, Vy: float, Vw: float) -> Twist:
        msg = Twist

        msg.linear.x = Vx
        msg.linear.y = Vy
        msg.linear.z = 0
        
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = Vw

        return msg
    
    def twist_stamper(self, twist_msg: Twist) -> TwistStamped:
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist = twist_msg

        return msg

    # Pulse to Wheel's Velocities Converter
    def pulse_to_wheel_vel(self, encoder_msg: list[float]) -> WheelsVelocity3:
        raise NotImplemented('Konversi pulse ke kecepatan roda')
    
    # Wheel's Velocities to Cmd Velocity Converter
    def wheel_vel_to_cmd_vel(self, wheel_vel: WheelsVelocity3) -> Twist:
        Vm1 = wheel_vel.wheel1
        Vm2 = wheel_vel.wheel2
        Vm3 = wheel_vel.wheel3

        Vx = ((3**0.5)/2)*Vm1 - ((3**0.5)/2)*Vm2
        Vy = -0.5*Vm1 - 0.5*Vm2 +Vm3
        Vw = Vm1 + Vm2 + Vm3

        return self.cmd_vel_twist(Vx, Vy, Vw)

    # cmd_vel Info
    def cmd_vel_info(self):
        msg = '\n'.join([
            f'vx: {self.cmd_vel_.twist.linear.x}',
            f'vy: {self.cmd_vel_.twist.linear.y}',
            f'vw: {self.cmd_vel_.twist.angular.z}'
        ])
        # self.get_logger().info(msg)
        print('---', 'cmd_vel:', msg, sep='\n')

    # Subscribe Callback
    def encoder_callback(self, msg: EncoderPulse):
        self.wheel_vel_ = self.pulse_to_wheel_vel(
            [msg.wheel1, msg.wheel2, msg.wheel3])
        cmd_vel = self.wheel_vel_to_cmd_vel(self.wheel_vel_)
        self.cmd_vel_ = self.twist_stamper(cmd_vel)

    # Publish Robot Velocity
    def publish_messages(self):
        self.cmd_vel_info()
        self.publisher_.publish(self.cmd_vel_)


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
