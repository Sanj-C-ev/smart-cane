import rclpy
import numpy as np
import serial
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import tf_transformations

class OccupancyGridMapper(Node):
    def __init__(self):
        super().__init__('occupancy_mapper')

        # Serial connection to ESP32
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Subscribe to odometry data
        self.odom_subscription = self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)

        # Publisher for occupancy grid
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)

        # Map settings
        self.grid_size = 100  # 100x100 cells
        self.resolution = 0.1  # Each cell = 10cm
        self.map_data = np.full((self.grid_size, self.grid_size), -1)  # -1 = unknown

        # Robot position (updated from odometry)
        self.robot_x = self.grid_size // 2
        self.robot_y = self.grid_size // 2
        self.robot_theta = 0.0  # Yaw angle

        # Timer to update map
        self.timer = self.create_timer(0.1, self.update_map)

    def odom_callback(self, msg):
        """ Updates robot position from odometry """
        self.robot_x = int(msg.pose.pose.position.x / self.resolution) + self.grid_size // 2
        self.robot_y = int(msg.pose.pose.position.y / self.resolution) + self.grid_size // 2

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    def mark_obstacle(self, x, y, size=10):
        """ Marks a larger area around an obstacle """
        for dx in range(-size, size + 1):
            for dy in range(-size, size + 1):
                new_x = x + dx
                new_y = y + dy

                if 0 <= new_x < self.grid_size and 0 <= new_y < self.grid_size:
                    self.map_data[new_x, new_y] = 100  # Mark as occupied

    def update_map(self):
        """ Reads sensor data, updates the occupancy grid, and publishes the map """
        try:
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            distances = list(map(float, line.split(',')))
            angles = [0, 45, -45]  # Front, left, right

            for i, distance in enumerate(distances):
                if 5 < distance < 100:  # Ignore very close and far readings
                    angle_rad = np.radians(angles[i]) + self.robot_theta

                    # Compute obstacle position in map frame
                    x = int(self.robot_x + (distance / self.resolution) * np.cos(angle_rad))
                    y = int(self.robot_y + (distance / self.resolution) * np.sin(angle_rad))

                    # Mark a bigger obstacle area
                    self.mark_obstacle(x, y, size=2)  # Increase size to 2 (5x5 block)

            self.publish_map()
        except Exception as e:
            self.get_logger().error(f"Error reading serial data: {e}")

    def publish_map(self):
        """ Publishes the occupancy grid map """
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size

        grid_msg.data = self.map_data.flatten().tolist()

        self.map_publisher.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
