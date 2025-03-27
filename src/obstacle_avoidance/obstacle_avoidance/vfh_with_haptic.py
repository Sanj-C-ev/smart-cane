import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

# UDP Configuration for ESP32
ESP32_IP = "10.42.0.161"  # Replace with actual ESP32 IP
UDP_PORT = 4210

class HapticUDPNode(Node):
    def __init__(self):
        super().__init__('haptic_udp_node')
        
        # ROS2 Subscription
        self.haptic_sub = self.create_subscription(String, '/haptic_feedback', self.haptic_callback, 10)
        
        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info("Haptic UDP Node Initialized.")

    def haptic_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received Haptic Command: {command}")
        
        # Send UDP command
        self.sock.sendto(command.encode(), (ESP32_IP, UDP_PORT))
        self.get_logger().info(f"Sent UDP Command: {command}")


def main():
    rclpy.init()
    node = HapticUDPNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
