import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import struct
import tf_transformations  # Import this at the top

class PointCloudMapper(Node):
    def __init__(self):
        super().__init__('pointcloud_mapper')

        # Subscriptions
        self.subscription = self.create_subscription(String, 'ultrasonic_data', self.sensor_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom_rf2o', self.odom_callback, 10)

        # Publisher
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'point_cloud', 10)

        # Robot position (updated from odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # Robot's orientation (yaw)

    def odom_callback(self, msg):
        """ Updates the robot's position and orientation from odometry """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion using tf_transformations
        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        _, _, ang = tf_transformations.euler_from_quaternion(quaternion)
        self.robo_theta = np.radians(ang)
        # Convert quaternion to yaw

    def sensor_callback(self, msg):
        """ Converts ultrasonic sensor data to a point cloud """
        distances = list(map(float, msg.data.split(',')))
        angles = [0, 45, -45]  # Front, left, right

        points = []

        for i, distance in enumerate(distances):
            if 0 < distance < 100:  # Ignore out-of-range values
                local_angle = angles[i]
                
                # Convert to local robot coordinates
                x_local = distance * np.cos(local_angle)
                y_local = distance * np.sin(local_angle)

                # Transform to global coordinates using odometry
                x_global = self.robot_x + x_local * np.cos(self.robot_theta) - y_local * np.sin(self.robot_theta)
                y_global = self.robot_y + x_local * np.sin(self.robot_theta) + y_local * np.cos(self.robot_theta)

                # Store the transformed point (z = 0 for 2D mapping)
                points.append((x_global, y_global, 0.0))

        self.publish_pointcloud(points)

    def publish_pointcloud(self, points):
        """ Publishes the transformed points as a PointCloud2 message """
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = "map"
        cloud_msg.height = 1  # Unordered point cloud
        cloud_msg.width = len(points)
        cloud_msg.is_dense = False
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 floats (x, y, z) * 4 bytes each
        cloud_msg.row_step = cloud_msg.point_step * len(points)

        # Define point fields (x, y, z as float32)
        cloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Convert points to binary format
        cloud_msg.data = b"".join(struct.pack("fff", *point) for point in points)

        # Publish the PointCloud2 message
        self.pointcloud_publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
