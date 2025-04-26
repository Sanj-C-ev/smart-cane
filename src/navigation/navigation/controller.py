#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8MultiArray

class HapticFeedbackController(Node):
    def __init__(self):
        super().__init__('haptic_feedback_controller')

        # Subscriptions
        self.create_subscription(Int8MultiArray, '/haptic', self.haptic_obstacle_callback, 10)

        # Parameters
        self.declare_parameter('publish_rate', 10)  # Hz
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publisher
        self.haptic_pub = self.create_publisher(String, '/haptic_feedback', 10)

        # Rate control variables
        self.pending_haptic_msg = None

        # Timer for controlled publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_pending_messages)

        self.get_logger().info(f"Haptic Feedback Controller ready (publishing at {self.publish_rate} Hz)")

    def haptic_obstacle_callback(self, msg):
        """Process incoming haptic data and prepare feedback"""
        if len(msg.data) != 5:
            self.get_logger().error("Invalid haptic data format")
            return

        left, near_left, front, near_right, right = msg.data

        # Set motor intensities based on obstacles
        m1 = 200 if front == 1 else 0
        m2 = 200 if front == 1 else 0
        m3 = 200 if left == 1 or near_left == 1 else 0
        m4 = 200 if right == 1 or near_right == 1 else 0

        motor_cmd = f"M1={m1}&M2={m2}&M3={m3}&M4={m4}"

        feedback_msg = String()
        feedback_msg.data = motor_cmd
        self.pending_haptic_msg = feedback_msg

    def publish_pending_messages(self):
        """Publish messages at a fixed rate"""
        if self.pending_haptic_msg is not None:
            self.haptic_pub.publish(self.pending_haptic_msg)
            self.pending_haptic_msg = None

def main(args=None):
    rclpy.init(args=args)
    controller = HapticFeedbackController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
