#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, Pose2D
from omniwheels_interfaces.action import GoToGoal


class GoToGoalServer(Node):
    def __init__(self):
        super().__init__('go_to_goal_server')

        # Constructor
        self.robot_pose_ = Pose2D()
        self.distance_tolerance_ = 10

        # Topics and Action Server
        self.subcsriber_ = self.create_subscription(
            Pose2D, 'robot_pose', self.subscriber_callback, 10)
        self.publisher_ = self.create_publisher(
            Twist, 'cmd_vel', 10
        )
        self.action_server = ActionServer(
            self, GoToGoal, 'go_to_goal', self.execute_callback)
        
        # Rate
        self.rate_ = self.create_rate(10)
        
        # Node Starting Info
        self.get_logger().info(f'Node has been started')


    # Calculate Distance Beetwen Current Pose and Goal Pose
    def euclidean_distance(self, goal_pose) -> float:
        inner1 = (goal_pose.x - self.robot_pose_.x) ** 2
        inner2 = (goal_pose.y - self.robot_pose_.y) ** 2
        return (inner1 + inner2) ** 0.5


    # Calculate Linear and Angular Velocity
    def velocity(
            self, goal: float, robot: float, maxi: float = 1.0) -> float:
        result = float(goal - robot)

        if result > maxi:
            return maxi
        else:
            return result


    # Subscriber Callback
    def subscriber_callback(self, msg: Pose2D):
        self.robot_pose_ = msg


    # Action Callback
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        goal_handle.succeed()
        result = GoToGoal.Result()
        return result

    # def execute_callback(self, goal_handle):
    #     self.get_logger().info('Getting a request')

    #     goal_pose: Pose2D = goal_handle.request.pose
    #     distance = self.euclidean_distance(goal_pose)
    #     feedback_msg = GoToGoal.Feedback()

    #     cmd_vel = Twist()

    #     while distance > self.distance_tolerance_:
    #         cmd_vel.linear.x = self.velocity(goal_pose.x, self.robot_pose_.x)
    #         cmd_vel.linear.y = self.velocity(goal_pose.y, self.robot_pose_.y)
    #         cmd_vel.linear.z = 0.0

    #         cmd_vel.angular.x = 0.0
    #         cmd_vel.angular.x = 0.0
    #         cmd_vel.angular.z = self.velocity(goal_pose.theta,
    #                                           self.robot_pose_.theta, 2.0)

    #         distance = self.euclidean_distance(goal_pose)
    #         feedback_msg.distance = distance
    #         goal_handle.publish_feedback(feedback_msg)
    #         self.publisher_.publish(cmd_vel)
    #         self.get_logger().info(f'{distance}')
    #         self.rate_.sleep()

    #     goal_handle.succeed()
    #     result = GoToGoal.Result()
    #     result.message = "Robot has arrived to desired goal position"
    #     self.get_logger().info('Successfully executed action')



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
