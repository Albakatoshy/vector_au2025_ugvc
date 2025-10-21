import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DiffDriveCmdVelConverter(Node):
    def __init__(self):
        super().__init__('diff_drive_cmd_vel_converter')

       
        self.wheel_separation = 0.2  
        self.wheel_radius = 0.05     

        
        self.subscription = self.create_subscription(
            Twist,'/diff_cont/cmd_vel_unstamped',self.cmd_vel_callback,10
        )
        self.left_pub = self.create_publisher(Float64, '/left_wheel_velocity_controller/command', 10)
        self.right_pub = self.create_publisher(Float64, '/right_wheel_velocity_controller/command', 10)
        self.get_logger().info("DiffDriveCmdVelConverter Node has started")

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        
        v_left = (linear_vel - (angular_vel * self.wheel_separation / 2.0)) / self.wheel_radius
        v_right = (linear_vel + (angular_vel * self.wheel_separation / 2.0)) / self.wheel_radius

        self.left_pub.publish(Float64(data=v_left))
        self.right_pub.publish(Float64(data=v_right))

        self.get_logger().info(f"cmd_vel: linear={linear_vel}, angular={angular_vel} -> left={v_left:.2f}, right={v_right:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveCmdVelConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
