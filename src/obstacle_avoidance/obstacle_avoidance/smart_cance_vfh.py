import rclpy
import numpy as np
import requests
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry  
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker
import tf_transformations

ESP32_HTTP_URL = "http://10.42.0.161/haptic"  # Change to your ESP32 IP

class SmartCaneVFH(Node):
    def __init__(self):
        super().__init__('smart_cane_vfh')

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)

        self.haptic_pub = self.create_publisher(String, '/haptic_feedback', 10)
        self.distance_pub = self.create_publisher(String, '/obstacle_distance', 10)
        self.direction_pub = self.create_publisher(Marker, '/direction_marker', 10)
        self.obstacle_pub = self.create_publisher(PointCloud2, '/obstacle_points', 10)

        self.declare_parameter('safe_distance', 1.0)
        self.safe_distance = self.get_parameter('safe_distance').value
        self.current_velocity = 0.0  

        self.get_logger().info("Smart Cane VFH Node Initialized.")

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x  

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges)] = msg.range_max  
        angle_range = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        obstacle_points = [[r * np.cos(theta), r * np.sin(theta), 0] 
                           for r, theta in zip(ranges, angle_range) if r < self.safe_distance]
        self.publish_obstacle_points(obstacle_points, msg.header)

        histogram = np.where(ranges < self.safe_distance, 1, 0)
        gap_sizes = np.diff(np.where(np.concatenate(([1], histogram, [1])))[0]) - 1

        if len(gap_sizes) == 0:
            self.send_haptic_command("STOP", intensity=255)
            return

        largest_gap_idx = np.argmax(gap_sizes)
        best_index = np.where(histogram == 0)[0][largest_gap_idx // 2]
        best_angle = angle_range[best_index]

        self.publish_direction_marker(best_angle, msg.header)
        self.get_logger().info(f"Best direction: {best_angle:.2f} radians")

        min_distance = np.min(ranges)
        adjusted_velocity = self.current_velocity * (min_distance / (self.safe_distance * 1.5)) if min_distance < self.safe_distance * 1.5 else self.current_velocity

        if best_angle > 0.1:
            self.send_haptic_command("RIGHT", self.calculate_intensity(min_distance, adjusted_velocity))
        elif best_angle < -0.1:
            self.send_haptic_command("LEFT", self.calculate_intensity(min_distance, adjusted_velocity))
        else:
            self.send_haptic_command("STRAIGHT", self.calculate_intensity(min_distance, adjusted_velocity))

    def calculate_intensity(self, min_distance, velocity):
        return int(min(255, max(100, (self.safe_distance - min_distance) * 300 + velocity * 100)))

    def publish_direction_marker(self, best_angle, header):
        marker = Marker()
        marker.header = header
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0

        yaw = -best_angle  
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)
        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = quaternion

        marker.scale.x = 0.5  
        marker.scale.y = marker.scale.z = 0.1
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.0, 1.0, 1.0  

        self.direction_pub.publish(marker)

    def publish_obstacle_points(self, points, header):
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.obstacle_pub.publish(cloud_msg)

    def send_haptic_command(self, direction, intensity=255):
        motor_map = {
            "LEFT": f"M1:0 M2:0 M3:{intensity} M4:0",
            "STRAIGHT": f"M1:{intensity} M2:{intensity} M3:0 M4:0",
            "RIGHT": f"M1:0 M2:0 M3:0 M4:{intensity}",
            "STOP": f"M1:{intensity} M2:{intensity} M3:{intensity} M4:{intensity}",
        }
        command = motor_map[direction]

        try:
            response = requests.post(ESP32_HTTP_URL, json={"command": command})
            if response.status_code == 200:
                self.get_logger().info(f"Sent HTTP Command: {command}")
            else:
                self.get_logger().error(f"ESP32 Error: {response.status_code} - {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"HTTP Request failed: {e}")

        self.haptic_pub.publish(String(data=f"{direction} - Intensity {intensity}"))


def main():
    rclpy.init()
    node = SmartCaneVFH()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

