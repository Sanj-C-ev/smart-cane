import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        # Set serial port and baud rate
        self.serial_port = "/dev/ttyUSB1"
        self.baud_rate = 115200

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to IMU on {self.serial_port}')
        except serial.SerialException:
            self.get_logger().error(f'Failed to connect to {self.serial_port}')
            exit(1)

        self.timer = self.create_timer(0.01, self.publish_imu_data)  # 100Hz

    def publish_imu_data(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()

            # Debug: Print raw data
            self.get_logger().info(f'Received: {line}')

            # Skip empty or malformed lines
            if not line or ',' not in line:
                self.get_logger().warn('Skipping invalid IMU data (empty or no commas)')
                return

            data = line.split(',')

            if len(data) != 6:
                self.get_logger().warn(f'Invalid data format: {line}')
                return

            try:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = map(float, data)
            except ValueError:
                self.get_logger().warn(f'Non-numeric IMU data: {line}')
                return

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # IMU Data
            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

            # **Updated Covariance Matrices** (Diagonal only)
            imu_msg.linear_acceleration_covariance = [
                2.7614625759999997e-05, 1e-6, 1e-6,
                1e-6, 2.2868475110000003e-05, 1e-6,
                1e-6, 1e-6, 6.673383999000121e-05
            ]

            imu_msg.angular_velocity_covariance = [
                7.489596000000002e-08, 1e-6, 1e-6,
                1e-6, 8.792630999999995e-08, 1e-6,
                1e-6, 1e-6, 6.018096e-08
            ]

            imu_msg.orientation_covariance = [-1.0] * 9  # Unknown orientation

            self.publisher_.publish(imu_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading IMU: {e}")

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()