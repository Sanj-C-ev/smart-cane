import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Pose2D
from visualization_msgs.msg import Marker
import math
import numpy as np

class PurePursuitPlanner(Node):
    def __init__(self):
        super().__init__('pure_pursuit_planner')

        # Initialize logger
        self.logger = self.get_logger()

        # Subscribe to the robot's global path and current pose
        self.create_subscription(Path, '/smart_cane_path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/smart_cane_pose', self.pose_callback, 10)

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lookahead_pub = self.create_publisher(Marker, '/lookahead_point', 10)

        # Robot state initialization
        self.robot_pose = Pose2D()
        self.robot_pose.x = 0.0
        self.robot_pose.y = 0.0
        self.robot_pose.theta = 0.0

        # Path storage
        self.global_path = []
        self.path_index = 0

        # Pure Pursuit parameters
        self.lookahead_distance = 0.5  # Distance to look ahead on the path (meters)
        self.max_linear_speed = 0.5     # Maximum linear speed (m/s)
        self.max_angular_speed = 1.0    # Maximum angular speed (rad/s)
        self.k_p = 0.5                  # Proportional gain for angular velocity
        self.goal_tolerance = 0.1       # Distance tolerance for reaching goal (m)
        self.control_rate = 10.0        # Control loop frequency (Hz)

        # Create timer for control loop
        self.create_timer(1.0/self.control_rate, self.control_loop)

    def path_callback(self, msg):
        """ Callback to update the global path """
        self.global_path = msg.poses
        self.path_index = 0
        self.logger.info(f"Received new path with {len(self.global_path)} waypoints")

    def pose_callback(self, msg):
        """ Callback to update the robot's current pose """
        self.robot_pose.x = msg.pose.position.x
        self.robot_pose.y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.robot_pose.theta = self.quaternion_to_euler(q.x, q.y, q.z, q.w)

    def quaternion_to_euler(self, x, y, z, w):
        """ Convert quaternion to Euler angles (roll, pitch, yaw) """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def find_lookahead_point(self):
        """ Find the lookahead point along the path """
        if not self.global_path or self.path_index >= len(self.global_path):
            return None

        # Start searching from current path index
        for i in range(self.path_index, len(self.global_path)):
            point = self.global_path[i].pose.position
            dx = point.x - self.robot_pose.x
            dy = point.y - self.robot_pose.y
            distance = math.sqrt(dx**2 + dy**2)

            # If we've reached the end of the path
            if i == len(self.global_path) - 1 and distance < self.goal_tolerance:
                return None

            # If this point is beyond the lookahead distance
            if distance >= self.lookahead_distance:
                self.path_index = i
                self.publish_lookahead_marker(point.x, point.y)
                return point

        # If all remaining points are within lookahead distance, use the last point
        self.path_index = len(self.global_path) - 1
        last_point = self.global_path[-1].pose.position
        self.publish_lookahead_marker(last_point.x, last_point.y)
        return last_point

    def publish_lookahead_marker(self, x, y):
        """ Publish a marker for visualization of the lookahead point """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.lookahead_pub.publish(marker)

    def compute_pure_pursuit_control(self, lookahead_point):
        """ Compute velocity commands using Pure Pursuit """
        if not lookahead_point:
            return Twist()  # Zero velocity if no lookahead point

        # Transform lookahead point to robot frame
        dx = lookahead_point.x - self.robot_pose.x
        dy = lookahead_point.y - self.robot_pose.y
        target_in_robot_frame = self.transform_to_robot_frame(dx, dy)

        # Pure Pursuit control law
        curvature = 2.0 * target_in_robot_frame[1] / (self.lookahead_distance ** 2)
        angular_vel = self.k_p * curvature * self.max_linear_speed
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)

        # Create velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = self.max_linear_speed
        cmd_vel.angular.z = angular_vel

        return cmd_vel

    def transform_to_robot_frame(self, x_world, y_world):
        """ Transform world coordinates to robot frame """
        # Rotation matrix
        cos_th = math.cos(self.robot_pose.theta)
        sin_th = math.sin(self.robot_pose.theta)
        
        # Transform to robot frame
        x_robot = x_world * cos_th + y_world * sin_th
        y_robot = -x_world * sin_th + y_world * cos_th
        
        return x_robot, y_robot

    def control_loop(self):
        """ Main control loop """
        if not self.global_path:
            self.logger.warn("No global path available")
            return

        # Check if we've reached the goal
        if self.path_index >= len(self.global_path) - 1:
            last_point = self.global_path[-1].pose.position
            dx = last_point.x - self.robot_pose.x
            dy = last_point.y - self.robot_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.goal_tolerance:
                self.logger.info("Goal reached!")
                self.vel_pub.publish(Twist())  # Stop the robot
                return

        # Find lookahead point
        lookahead_point = self.find_lookahead_point()
        if not lookahead_point:
            self.logger.info("No valid lookahead point")
            self.vel_pub.publish(Twist())  # Stop the robot
            return

        # Compute velocity using Pure Pursuit
        cmd_vel = self.compute_pure_pursuit_control(lookahead_point)
        
        # Publish velocity command
        self.vel_pub.publish(cmd_vel)
        self.logger.info(f"Published cmd_vel: linear={cmd_vel.linear.x:.2f}, angular={cmd_vel.angular.z:.2f}")

def main():
    rclpy.init()
    node = PurePursuitPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()