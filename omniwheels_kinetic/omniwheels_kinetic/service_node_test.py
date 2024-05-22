#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ServiceNode(Node):

    def __init__(self):
        super().__init__('ball_shoot_srv_test')
        self.srv = self.create_service(Trigger, 'ball_shoot', self.shooting_callback)
        self.get_logger().info('Node service has been started')

    def shooting_callback(
            self, request: Trigger.Request, 
            response: Trigger.Response
            ) -> Trigger.Response:
        self.get_logger().info('Incoming request: Robot takes a shot!')
        response.success = True
        response.message = 'Robot shoots successfully'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode has been stopped')
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()