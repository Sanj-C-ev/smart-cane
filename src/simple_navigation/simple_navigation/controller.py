#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8MultiArray

class HapticFeedbackController(Node):
    def __init__(self):
        super().__init__('haptic_feedback_controller')

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int8MultiArray, '/haptic', self.haptic_obstacle_callback, 10)

        # Parameters
        self.declare_parameter('max_linear_vel', 1.0)  # m/s
        self.declare_parameter('max_angular_vel', 0.5)  # rad/s
        self.declare_parameter('min_angle', 30)  # degrees
        self.declare_parameter('max_angle', 150)  # degrees
        self.declare_parameter('publish_rate', 10)  # Hz

        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        #self.kinesthetic_pub = self.create_publisher(String, '/kinesthetic_feedback', 10)
        self.haptic_pub = self.create_publisher(String, '/haptic_feedback', 10)

        # Rate control variables
        self.last_publish_time = self.get_clock().now()
        self.pending_kinesthetic_msg = None
        self.pending_haptic_msg = None

        # Timer for rate-controlled publishing
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_pending_messages)

        self.get_logger().info(f"Haptic Feedback Controller ready (publishing at {self.publish_rate}Hz)")
        self.rate = self.create_rate(10)  # 10 Hz

    def cmd_vel_callback(self, msg):
        """Handle kinesthetic feedback from cmd_vel"""
        try:
            self.process_kinesthetic_feedback(msg)
        except Exception as e:
            self.get_logger().error(f"Error processing cmd_vel: {str(e)}")

    def process_kinesthetic_feedback(self, msg):
        """Store servo and motor commands for rate-controlled publishing"""
        angular_vel = msg.angular.z
        angle = self.map_angular_to_angle(angular_vel)

        linear_vel = msg.linear.x
        motor_value = self.map_linear_to_motor(linear_vel)

        command_str = f"s:{angle},m:{motor_value}"
        feedback_msg = String()
        feedback_msg.data = command_str
        self.pending_kinesthetic_msg = feedback_msg
        self.rate.sleep()  # This will ensure the loop runs at 10 Hz

    def haptic_obstacle_callback(self, msg):
        """Haptic motor feedback based on obstacle positions"""
        if len(msg.data) != 5:
            self.get_logger().error("Invalid haptic data format")
            return

        left, near_left, front, near_right, right = msg.data

        # Set intensities based on obstacle presence
        m1 = 200 if front == 1 else 0
        m2 = 200 if front == 1 else 0
        m3 = 200 if left == 1 or near_left == 1 else 0
        m4 = 200 if right == 1 or near_right == 1 else 0

        motor_cmd = f"M1={m1}&M2={m2}&M3={m3}&M4={m4}"
        feedback_msg = String()
        feedback_msg.data = motor_cmd
        self.pending_haptic_msg = feedback_msg
        s = 90
        # Steering logic based on obstacles
        if front == 0:
            s = 90
        else:
            if near_left == 1 and left == 1:
                s = 60
            elif left == 1 and right == 1:
                s = 120

        # Update kinesthetic feedback if obstacle requires steering change
        command_str = f"s:{s},m:{self.map_linear_to_motor(0)}"  # Stop motor when obstacle detected
        steering_msg = String()
        steering_msg.data = command_str
        self.pending_kinesthetic_msg = steering_msg
        self.rate.sleep()  # This will ensure the loop runs at 10 Hz
        
    def publish_pending_messages(self):
        """Publish pending messages at the controlled rate"""
        current_time = self.get_clock().now()
        
        #if self.pending_kinesthetic_msg is not None:
         #   self.kinesthetic_pub.publish(self.pending_kinesthetic_msg)
          #  self.pending_kinesthetic_msg = None
            
        if self.pending_haptic_msg is not None:
            self.haptic_pub.publish(self.pending_haptic_msg)
            self.pending_haptic_msg = None

    def map_angular_to_angle(self, angular_vel):
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        normalized = angular_vel / self.max_angular_vel
        angle = 90 + (normalized * 30)
        return int(round(max(self.min_angle, min(angle, self.max_angle))))

    def map_linear_to_motor(self, linear_vel):
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