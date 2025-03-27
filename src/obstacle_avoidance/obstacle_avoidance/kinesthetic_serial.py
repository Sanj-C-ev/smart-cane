import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32  # Publisher for debugging
import serial
import math

class PathToServo(Node):
    def __init__(self):
        super().__init__('path_to_servo')

        # Set up serial connection (Uncomment if using serial)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        self.plan_subscription = self.create_subscription(
            Path, 
            '/plan', 
            self.path_callback, 
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry, 
            '/odom_rf2o', 
            self.odom_callback, 
            10
        )

        self.servo_angle_publisher = self.create_publisher(Float32, '/servo_angle', 10)

        # Default position and heading
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0  # Heading in radians

    def map_value(self, value, fmin, fmax, tmin, tmax):
        """Maps value from one range to another."""
        return tmin + (value - fmin) * (tmax - tmin) / (fmax - fmin)

    def quaternion_to_yaw(self, q):
        """Converts a quaternion to yaw (rotation around Z-axis in radians)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        """Updates the current position and heading from /odom_rf2o."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)  

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            self.get_logger().warn("No valid path received!")
            return

        next_x = msg.poses[1].pose.position.x
        next_y = msg.poses[1].pose.position.y

        theta_target = math.atan2(next_y - self.current_y, next_x - self.current_x)

        theta_diff = theta_target - self.current_yaw

        # Normalize angle to range [-π, π]
        theta_diff = (theta_diff + math.pi) % (2 * math.pi) - math.pi

        # Convert to degrees
        theta_deg = math.degrees(theta_diff)

        # Map θ_diff (-90° to 90°) to servo angle (45° to 135°)
        servo_angle = self.map_value(theta_deg, -90, 90, 45, 135)
        servo_angle = max(45, min(135, int(servo_angle)))  # Ensure within range

        # Send servo angle to ESP32 (Uncomment if using serial)
        data = f"{servo_angle}\n"
        self.ser.write(data.encode())  

        # Publish servo angle for debugging
        angle_msg = Float32()
        angle_msg.data = float(servo_angle)
        self.servo_angle_publisher.publish(angle_msg)

        self.get_logger().info(f"Next waypoint: ({next_x:.2f}, {next_y:.2f}) | Target θ: {math.degrees(theta_target):.1f}° | "
                               f"Current θ: {math.degrees(self.current_yaw):.1f}° | θ_diff: {theta_deg:.1f}° | Servo: {servo_angle}")

def main(args=None):
    rclpy.init(args=args)
    node = PathToServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
