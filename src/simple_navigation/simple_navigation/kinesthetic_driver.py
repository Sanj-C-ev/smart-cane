#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class KinestheticFeedbackNode(Node):
    def __init__(self):
        super().__init__('kinesthetic_feedback_node')
        
        self.declare_parameters('', [
            ('serial_port', '/dev/ttyUSB1'),
            ('serial_baud', 115200),
            ('serial_timeout', 1.0),
        ])
        
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('serial_baud').value
        self.timeout = self.get_parameter('serial_timeout').value
        self.serial_conn = None

        self.subscription = self.create_subscription(
            String,
            '/kinesthetic_feedback',
            self.kinesthetic_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.init_serial()
        self.get_logger().info("Kinesthetic Feedback Node Initialized")

    def init_serial(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2)
            self.get_logger().info(f"Serial connected on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial connection error: {str(e)}")
            self.serial_conn = None

    def kinesthetic_callback(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            self.get_logger().warn("Empty kinesthetic command")
            return

        if not cmd.endswith('\n'):
            cmd += '\n'

        if not self.serial_conn or not self.serial_conn.is_open:
            self.init_serial()
            if not self.serial_conn:
                self.get_logger().error("Serial unavailable")
                return

        try:
            self.serial_conn.write(cmd.encode())
            self.get_logger().info(f"Sent kinesthetic: {cmd.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {str(e)}")
            self.init_serial()

    def __del__(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed")

def main():
    rclpy.init()
    node = KinestheticFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
