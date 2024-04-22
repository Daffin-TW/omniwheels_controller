#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from omniwheels_interfaces.msg import UWBAnchor


class UWB(Node):
    def __init__(self):
        super().__init__('uwb_listener')

        # Constructor
        self.distance_anchor_1_2_ = 800  # Distance between anchor 1 and anchor 2
        self.tag_position_ = (float(), float())
        self.anchor_1_ = float()
        self.anchor_2_ = float()
        self.x_ = float()
        self.y_ = float()

        # Topics
        self.subscriber_ = self.create_subscription(UWBAnchor, 'uwb_topic', self.subscriber_callback, 10)
        self.publisher_ = self.create_publisher(Point32, 'uwb_coordinate', 10)

        # Timer
        self.timer_ = self.create_timer(0.1, self.publisher_callback)

        
    # Getting Tag Position
    def tag_pos(self, da: float, db: float, dab: float) -> tuple[float]:
        if da + db < dab:
            self.get_logger().info(
                '\nJumlah jarak tag ke anchor a dengan jarak tag ke anchor b ' +
                'kurang dari jarak anchor a ke anchor b, ' +
                f'{da} + {db} = {da + db} < {dab}'
            )
        elif abs(da - db) > dab:
            self.get_logger().info(
                '\nMutlak Selisih jarak tag ke anchor a dengan jarak tag ' +
                'ke anchor b lebih dari jarak anchor a ke anchor b, ' +
                f'|{da} + {db}| = |{(da - db)}| > {dab}'
            )
        else:
            cos_a = (da**2 + dab**2 - db**2) / (2*da*dab)
            x = da*cos_a
            y = da*((1 - cos_a**2)**0.5)
            return x, y
        
    
    # Subscriber Callback
    def subscriber_callback(self, msg: UWBAnchor):
        self.anchor_1_ = msg.anchor1
        self.anchor_2_ = msg.anchor2


    # Publisher Callback
    def publisher_callback(self):
        self.x_, self.y_ = self.tag_pos(self.anchor_1_, self.anchor_2_, self.distance_anchor_1_2_)
        point = Point32()
        point.x = self.x_
        point.y = self.y_
        self.get_logger().info(f'Robot Coordinate: ({self.x_}, {self.y_})')
        self.publisher_.publish(point)


def main(args=None):
    rclpy.init(args=args)
    node = UWB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode has been stopped')
    else:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
