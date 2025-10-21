import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthToOccupancyMap(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        # Map parameters
        self.resolution = 0.05  # meters per cell
        self.width = 200        # cells
        self.height = 200
        self.origin_x = -5.0
        self.origin_y = -5.0

        # Camera intrinsics (adjust if needed)
        self.fx = 554.254
        self.fy = 554.254
        self.cx = 320.5
        self.cy = 240.5

        # Bridge for image conversion
        self.bridge = CvBridge()

        # QoS to match what Nav2 and RViz expect for /map
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # ROS interfaces
        self.publisher = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        self.subscription = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Rate limiter
        self.last_publish_time = self.get_clock().now()

    def depth_callback(self, msg: Image):
        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds < 1e9:
            return
        self.last_publish_time = now

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CVBridge error: {e}")
            return

        # Convert to meters if in mm
        if depth_image.dtype != np.float32:
            depth_image = depth_image.astype(np.float32) / 1000.0

        # Create occupancy grid
        occupancy_grid = np.zeros((self.height, self.width), dtype=np.int8)

        for v in range(depth_image.shape[0]):
            for u in range(depth_image.shape[1]):
                z = depth_image[v, u]
                if np.isnan(z) or z <= 0.2 or z >= 3.0:
                    continue

                x = (u - self.cx) * z / self.fx
                y = (v - self.cy) * z / self.fy

                mx = int((x - self.origin_x) / self.resolution)
                my = int((y - self.origin_y) / self.resolution)

                if 0 <= mx < self.width and 0 <= my < self.height:
                    occupancy_grid[my, mx] = 100  # mark as occupied

        # Fill OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = now.to_msg()
        map_msg.header.frame_id = 'map'

        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = occupancy_grid.flatten().tolist()
        self.publisher.publish(map_msg)
        self.get_logger().info("Published depth-based occupancy map")

def main(args=None):
    rclpy.init(args=args)
    node = DepthToOccupancyMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
