#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import math


class RvizStraightPathPublisher(Node):
    def __init__(self):
        super().__init__('rviz_straight_path_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, '/a_star/path', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.num_points = 50
        self.get_logger().info("✅ Ready for 2D Nav Goal in RViz")

    def goal_callback(self, msg: PoseStamped):
        try:
            now = rclpy.time.Time()
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', now, timeout=rclpy.duration.Duration(seconds=1.0)
            )

            start_x = tf.transform.translation.x
            start_y = tf.transform.translation.y

            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y

            self.get_logger().info(f"🟢 Start: ({start_x:.2f}, {start_y:.2f}) → Goal: ({goal_x:.2f}, {goal_y:.2f})")

            self.publish_straight_path(start_x, start_y, goal_x, goal_y)

        except Exception as e:
            self.get_logger().warn(f"❌ Could not get robot pose from TF: {e}")

    def publish_straight_path(self, start_x, start_y, goal_x, goal_y):
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(self.num_points + 1):
            t = i / self.num_points
            x = (1 - t) * start_x + t * goal_x
            y = (1 - t) * start_y + t * goal_y

            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # facing straight for now

            path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info("✅ Published straight path to /a_star/path")


def main(args=None):
    rclpy.init(args=args)
    node = RvizStraightPathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
