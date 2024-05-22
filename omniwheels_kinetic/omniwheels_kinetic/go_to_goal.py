#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, Pose2D
# from omniwheels_interfaces.action import GoToGoal


class GoToGoalServer(Node):
    def __init__(self):
        super().__init__('go_to_goal_server')

        # Constructor
        self.robot_pose_now_ = Pose2D()
        self.robot_pose_request_ = Pose2D()
        self.distance_tolerance_ = 0.1
        self.moving_ = False

        # Topics and Action Server
        self.sub_robot_pose_now_ = self.create_subscription(
            Pose2D, 'robot_pose_now', self.update_robot_position_now, 10)
        self.sub_robot_pose_request_ = self.create_subscription(
            Pose2D, 'robot_pose_request', self.update_robot_position_request, 10)
        self.publisher_ = self.create_publisher(
            Twist, 'cmd_vel', 10
        )
        self.debugger_timer_ = self.create_timer(1.0, self.position_debugger)
        self.velocity_timer_ = self.create_timer(0.1, self.velocity_publish)
        self.rate_ = self.create_rate(10)
        self.get_logger().info(f'Node has been started')

    # Calculate Distance Beetwen Current Pose and Goal Pose
    def euclidean_distance(self) -> float:
        inner1 = (self.robot_pose_request_.x - self.robot_pose_now_.x) ** 2
        inner2 = (self.robot_pose_request_.y - self.robot_pose_now_.y) ** 2
        inner3 = (self.robot_pose_request_.theta - self.robot_pose_now_.theta) ** 2
        return (inner1 + inner2 + inner3) ** 0.5

    # Calculate Linear and Angular Velocity
    def velocity(
            self, goal: float, robot: float, maximum_speed: float = 1.0
            ) -> float:
        result = float(goal - robot)

        if result > maximum_speed:
            return maximum_speed
        else:
            return result

    # Subscriber Callback
    def update_robot_position_now(self, msg: Pose2D):
        self.robot_pose_now_ = msg

    def update_robot_position_request(self, msg: Pose2D):
        if msg != self.robot_pose_request_:
            self.robot_pose_request_ = msg
            self.execute_go_to_goal()

    def position_debugger(self):
        print(
            'Robot Position Request:',
            f'- x: {self.robot_pose_request_.x}',
            f'- y: {self.robot_pose_request_.y}',
            f'- theta: {self.robot_pose_request_.theta}',
            'Robot Position Now:',
            f'- x: {self.robot_pose_now_.x}',
            f'- y: {self.robot_pose_now_.y}',
            f'- theta: {self.robot_pose_now_.theta}',
            f'Distance: {self.euclidean_distance()}',
            f'Is Moving: {self.moving_}',
            '---',
            sep='\n'
        )

    def execute_go_to_goal(self):
        if not self.moving_:
            self.moving_ = True
            self.get_logger().info('Getting a request')

    def velocity_publish(self):
        if not self.moving_:
            return

        distance = self.euclidean_distance()
        cmd_vel = Twist()

        if distance > self.distance_tolerance_:
            cmd_vel.linear.x = self.velocity(self.robot_pose_request_.x,
                                             self.robot_pose_now_.x)
            cmd_vel.linear.y = self.velocity(self.robot_pose_request_.y,
                                             self.robot_pose_now_.y)
            cmd_vel.linear.z = 0.0

            cmd_vel.angular.x = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.z = self.velocity(self.robot_pose_request_.theta,
                                              self.robot_pose_now_.theta, 2.0)

            distance = self.euclidean_distance()
            self.publisher_.publish(cmd_vel)
            self.rate_.sleep()

        else:
            self.moving_ = False
            self.get_logger().info('Successfully executed action')


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode has been stopped')
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
