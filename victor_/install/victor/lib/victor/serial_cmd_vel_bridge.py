#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist


class SerialCmdVelBridge(Node):
    def __init__(self):
        super().__init__('serial_cmd_vel_bridge')

        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Use /dev/serial0 for GPIO UART
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        serial_data = f"{linear},{angular}\n"
        try:
            self.serial_port.write(serial_data.encode('utf-8'))
            self.get_logger().info(f"Sent: {serial_data.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialCmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()    
