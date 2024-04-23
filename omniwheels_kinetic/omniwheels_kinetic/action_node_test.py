import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from omniwheels_interfaces.action import GoToGoal


class ActionClientNode(Node):

    def __init__(self):
        super().__init__('go_to_goal_client')
        self.action_client_ = ActionClient(self, GoToGoal, 'go_to_goal')
        self.get_logger().info('Node service has been started')

    def send_goal(self, pose):
        goal_msg = GoToGoal.Goal()
        goal_msg.pose = pose

        self.action_client_.wait_for_server()

        return self.action_client_.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = ActionClientNode()
    pose_msg = Pose2D()
    pose_msg.x = 20.0
    pose_msg.y = 25.0
    pose_msg.theta = 7.0

    future = action_client.send_goal(pose_msg)

    rclpy.spin_until_future_complete(action_client, future)

    print(future.result())


if __name__ == '__main__':
    main()