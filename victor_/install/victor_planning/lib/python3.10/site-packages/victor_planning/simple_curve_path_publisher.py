#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math


class CurvePathPublisher(Node):
    def __init__(self):
        super().__init__('curve_path_publisher')

        # Declare and get parameters
        self.declare_parameter('start', [0.0, 0.0])
        self.declare_parameter('goal', [2.0, 2.0])
        self.declare_parameter('num_points', 50)

        self.start = self.get_parameter('start').value
        self.goal = self.get_parameter('goal').value
        self.num_points = self.get_parameter('num_points').value

        # Publisher
        self.path_pub = self.create_publisher(Path, '/a_star/path', 10)

        # Timer to continuously publish path
        self.timer = self.create_timer(1.0, self.publish_curve_path)

    def publish_curve_path(self):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        start_x, start_y = self.start
        goal_x, goal_y = self.goal

        # Control point for the Bézier curve
        ctrl_x = (start_x + goal_x) / 2.0
        ctrl_y = (start_y + goal_y) / 2.0 + 1.0  # raised in y for curve

        for t in [i / self.num_points for i in range(self.num_points + 1)]:
            x = (1 - t)**2 * start_x + 2 * (1 - t) * t * ctrl_x + t**2 * goal_x
            y = (1 - t)**2 * start_y + 2 * (1 - t) * t * ctrl_y + t**2 * goal_y

            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # facing forward

            path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info('Published curved path.')


def main(args=None):
    rclpy.init(args=args)
    node = CurvePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
