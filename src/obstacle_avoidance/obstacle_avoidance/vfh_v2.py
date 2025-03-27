import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math
from visualization_msgs.msg import Marker

class VFHNavigator(Node):
    def __init__(self):
        super().__init__('vfh_navigator')

        # Subscribe to LiDAR scan
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Subscribe to Odometry (to get heading direction)
        self.odom_sub = self.create_subscription(Odometry, '/odom_rf2o', self.odom_callback, 10)

        # Publish safe direction for haptic feedback
        self.haptic_pub = self.create_publisher(Float32, '/haptic_feedback', 10)
        self.marker_pub = self.create_publisher(Marker, '/vfh_marker', 10)
        self.heading_angle = 0.0  # Stores the user's heading direction



    def publish_marker(self, angle):
        """ Publishes a visualization marker for RViz. """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "vfh_direction"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set arrow position & orientation
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = math.sin(angle / 2.0)
        marker.pose.orientation.w = math.cos(angle / 2.0)

        marker.scale.x = 0.5  # Arrow length
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


    def odom_callback(self, msg):
        """ Extracts heading direction (yaw) from odometry. """
        quaternion = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        self.heading_angle = yaw  # Store user's heading direction

    def quaternion_to_euler(self, x, y, z, w):
        """ Converts quaternion to Euler angles. """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw  # Return yaw as the heading angle

    def lidar_callback(self, msg):
        """ Processes LiDAR scan with VFH to find the safest direction. """
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter valid LiDAR readings
        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        # Compute polar histogram (obstacle density)
        num_bins = 72  # Divide 360° into 72 bins (5° per bin)
        bin_angles = np.linspace(-np.pi, np.pi, num_bins)
        histogram = np.zeros(num_bins)

        for i in range(len(ranges)):
            bin_index = np.digitize(angles[i], bin_angles) - 1
            histogram[bin_index] += 1  # Increase obstacle density

        # Identify largest free gap
        free_bins = np.where(histogram < 2)[0]
        if len(free_bins) == 0:
            self.get_logger().warn("No free path found!")
            return

        # Pick the middle of the largest free gap
        safe_angle_index = free_bins[len(free_bins) // 2]
        theta_safe = bin_angles[safe_angle_index]

        # Blend safe direction with user's heading
        blended_angle = self.blend_directions(self.heading_angle, theta_safe)
        blended_angle = np.degrees(blended_angle)
        # Publish haptic feedback
        msg = Float32()
        msg.data = float(blended_angle)
        self.haptic_pub.publish(msg)
        self.publish_marker(blended_angle)

        self.get_logger().info(f'Published Safe Direction: {blended_angle}°')

    def blend_directions(self, theta_heading, theta_safe):
        """ Blends the user's heading direction with the VFH safe direction. """
        alpha = 0.9  # 60% preference for user's heading, 40% for safe direction
        return alpha * theta_heading + (1 - alpha) * theta_safe

def main(args=None):
    rclpy.init(args=args)
    node = VFHNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
