#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class HapticFeedbackController(Node):
    def __init__(self):
        super().__init__('haptic_feedback_controller')
        
        # Subscribe to cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Parameters
        self.declare_parameter('max_linear_vel', 0.5)  # m/s
        self.declare_parameter('max_angular_vel', 1.0)  # rad/s
        self.declare_parameter('min_angle', 60)  # degrees
        self.declare_parameter('max_angle', 120)  # degrees
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        
        # Publishers
        self.kinesthetic_pub = self.create_publisher(String, '/kinesthetic_feedback', 10)
        self.haptic_pub = self.create_publisher(String, '/haptic_feedback', 10)
        
        self.get_logger().info("Haptic Feedback Controller ready")

    def cmd_vel_callback(self, msg):
        """Handle both kinesthetic and haptic feedback from cmd_vel"""
        try:
            # Process kinesthetic feedback (servo + motor)
            self.process_kinesthetic_feedback(msg)
            
            # Process haptic feedback (vibration motors)
            self.process_haptic_feedback(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing cmd_vel: {str(e)}")

    def process_kinesthetic_feedback(self, msg):
        """Publish servo and motor commands"""
        # Process angular velocity (steering/servo)
        angular_vel = msg.angular.z
        angle = self.map_angular_to_angle(angular_vel)
        
        # Process linear velocity (motor)
        linear_vel = msg.linear.x
        motor_value = self.map_linear_to_motor(linear_vel)
        
        # Create command string
        command_str = f"s:{angle},m:{motor_value}"
        
        # Publish for other nodes to use
        feedback_msg = String()
        feedback_msg.data = command_str
        self.kinesthetic_pub.publish(feedback_msg)
        
        self.get_logger().debug(f"Published kinesthetic: {command_str}")

    def process_haptic_feedback(self, msg):
        """Publish vibration motor commands in ESP32C3 format"""
        # Normalize velocities
        norm_angular = msg.angular.z / self.max_angular_vel
        norm_linear = msg.linear.x / self.max_linear_vel
        
        # Calculate direction intensities
        if norm_angular > 0:  # Right turn
            right_intensity = min(255, abs(norm_angular) * 255)
            left_intensity = 0
        else:  # Left turn
            left_intensity = min(255, abs(norm_angular) * 255)
            right_intensity = 0
        
        # Calculate front motors intensity (reduces with turning)
        front_intensity = min(255, norm_linear * 255 * (1 - abs(norm_angular)))
        
        # Create motor command string in ESP32C3 format (M1=value&M2=value...)
        motor_cmd = f"M1={int(front_intensity)}&M2={int(front_intensity)}&M3={int(left_intensity)}&M4={int(right_intensity)}"
        
        # Publish to haptic feedback topic
        feedback_msg = String()
        feedback_msg.data = motor_cmd
        self.haptic_pub.publish(feedback_msg)
        
        self.get_logger().debug(f"Published haptic: {motor_cmd}")

    def map_angular_to_angle(self, angular_vel):
        """Map angular velocity to servo angle (60-120)"""
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        normalized = angular_vel / self.max_angular_vel
        angle = 90 + (normalized * 30)  # ±30 degrees from center
        return int(round(max(self.min_angle, min(angle, self.max_angle))))

    def map_linear_to_motor(self, linear_vel):
        """Map linear velocity to motor value (255-0)"""
        linear_vel = max(0, min(linear_vel, self.max_linear_vel))
        normalized = linear_vel / self.max_linear_vel
        return 255 - int(round(normalized * 255))

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