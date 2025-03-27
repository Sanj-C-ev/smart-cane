import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import Int32
import math

class HapticFeedbackNode(Node):
    def __init__(self):
        super().__init__('haptic_feedback_node')
        self.subscription = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.publisher = self.create_publisher(Int32, '/haptic_feedback', 10)

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        
        current_pose = msg.poses[0].pose.position
        next_waypoint = msg.poses[1].pose.position
        angle = self.calculate_angle(current_pose, next_waypoint)
        haptic_value = int(angle / 2)  # Map to 0-180
        self.publisher.publish(Int32(data=haptic_value))

    def calculate_angle(self, current_pose, next_waypoint):
        delta_x = next_waypoint.x - current_pose.x
        delta_y = next_waypoint.y - current_pose.y
        angle_rad = math.atan2(delta_y, delta_x)
        angle_deg = math.degrees(angle_rad)
        return (angle_deg + 360) % 360

def main(args=None):
    rclpy.init(args=args)
    node = HapticFeedbackNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
