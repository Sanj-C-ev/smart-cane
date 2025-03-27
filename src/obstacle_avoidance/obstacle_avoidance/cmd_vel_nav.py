import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import collections
import math
import serial

class HapticFeedbackNode(Node):
    def __init__(self):
        super().__init__('haptic_feedback_node')
        
        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Serial communication with haptic device
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        # Sliding window for smoothing
        self.window_size = 20
        self.linear_vel_window = collections.deque(maxlen=self.window_size)
        self.angular_vel_window = collections.deque(maxlen=self.window_size)

    def cmd_vel_callback(self, msg):
        # Add velocities to the sliding window
        self.linear_vel_window.append(msg.linear.x)
        self.angular_vel_window.append(msg.angular.z)

        # Smooth the velocities (moving average)
        avg_linear_x = sum(self.linear_vel_window) / len(self.linear_vel_window)
        avg_angular_z = sum(self.angular_vel_window) / len(self.angular_vel_window)

        # Convert to direction angle (between 45° to 135°)
        scaling_factor = 20
        angle = 90 + avg_angular_z * scaling_factor
        angle = max(45, min(135, angle))

        # Send to haptic device
        self.send_to_haptic_device(angle)
        self.get_logger().info(f'Smoothed Angle: {angle}°')

    def send_to_haptic_device(self, angle):
        try:
            self.serial_port.write(f'{angle}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = HapticFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
