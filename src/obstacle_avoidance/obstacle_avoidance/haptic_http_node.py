import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

ESP32_HTTP_URL = "http://192.168.9.178/haptic"

class HapticHTTPNode(Node):
    def __init__(self):
        super().__init__('haptic_http_node')
        self.haptic_sub = self.create_subscription(String, '/haptic_feedback', self.haptic_callback, 10)
        self.get_logger().info("Haptic HTTP Node Initialized.")

    def haptic_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received Haptic Command: {command}")

        try:
            response = requests.post(ESP32_HTTP_URL, json={"command": command})
            if response.status_code == 200:
                self.get_logger().info(f"ESP32 acknowledged: {response.text}")
            else:
                self.get_logger().error(f"ESP32 error {response.status_code}: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send command: {e}")

def main():
    rclpy.init()
    node = HapticHTTPNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

