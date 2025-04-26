#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class HapticFeedbackNode(Node):
    def __init__(self):
        super().__init__('haptic_feedback_node')
        
        self.declare_parameters('', [
            ('http_ip', '192.168.9.178'),
            ('http_timeout', 1.5),
            ('http_retries', 3),
        ])
        
        self.url = f"http://{self.get_parameter('http_ip').value}/control"
        self.timeout = self.get_parameter('http_timeout').value
        self.retries = self.get_parameter('http_retries').value

        self.subscription = self.create_subscription(
            String,
            '/haptic_feedback',
            self.haptic_callback,
            10
        )
        self.get_logger().info("Haptic Feedback Node Initialized")

    def validate_haptic_command(self, command_str):
        parts = command_str.split('&')
        if len(parts) != 4:
            raise ValueError("Expected 4 motor commands (M1-M4)")

        for i, part in enumerate(parts):
            if not part.startswith(f'M{i+1}='):
                raise ValueError(f"Invalid format at motor M{i+1}")
            val = int(part.split('=')[1])
            if not (0 <= val <= 255):
                raise ValueError(f"Motor value out of range: {val}")

    def haptic_callback(self, msg):
        cmd = msg.data.strip()
        if not cmd:
            self.get_logger().warn("Empty haptic command")
            return

        try:
            self.validate_haptic_command(cmd)
        except ValueError as e:
            self.get_logger().error(str(e))
            return

        for attempt in range(self.retries):
            try:
                response = requests.post(
                    self.url,
                    data=cmd,
                    timeout=self.timeout,
                    headers={'Content-Type': 'application/x-www-form-urlencoded'}
                )
                if response.status_code == 200:
                    self.get_logger().info(f"Haptic sent: {response.text.strip()}")
                    return
                else:
                    self.get_logger().warn(f"HTTP {response.status_code}: {response.text.strip()}")
            except Exception as e:
                self.get_logger().warn(f"HTTP attempt {attempt+1} failed: {e}")
        self.get_logger().error("Failed to send haptic command after retries")

def main():
    rclpy.init()
    node = HapticFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
