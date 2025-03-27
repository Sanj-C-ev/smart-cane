import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

class ESP32SensorNode(Node):
    def __init__(self):
        super().__init__('esp32_sensor_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.ser.reset_input_buffer()  # 🔹 Clear any existing data in the buffer
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {serial_port}: {e}")
            return


        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.ultrasonic_cluster_1_publisher = self.create_publisher(Float32MultiArray, '/ultrasonic_cluster_1', 10)
        self.ultrasonic_cluster_2_publisher = self.create_publisher(Float32MultiArray, '/ultrasonic_cluster_2', 10)

        self.timer = self.create_timer(0.05, self.read_serial_data)  # 20Hz

    def read_serial_data(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                if not line:
                    self.get_logger().warn("Received empty line from ESP32.")
                    return

                parts = line.split(",")

                # Ensure data contains expected headers
                if "IMU" not in parts or "US1" not in parts or "US2" not in parts:
                    self.get_logger().warn(f"Invalid data format: {line}")
                    return

                try:
                    imu_index = parts.index("IMU") + 1
                    us1_index = parts.index("US1") + 1
                    us2_index = parts.index("US2") + 1

                    imu_data = [float(x) for x in parts[imu_index:imu_index + 6]]
                    us1_data = [float(x) for x in parts[us1_index:us1_index + 3]]
                    us2_data = [float(x) for x in parts[us2_index:us2_index + 3]]

                except (ValueError, IndexError):
                    self.get_logger().warn(f"Error parsing data: {line}")
                    return

                # Publish IMU Data
                imu_msg = Imu()
                imu_msg.linear_acceleration.x = imu_data[0]
                imu_msg.linear_acceleration.y = imu_data[1]
                imu_msg.linear_acceleration.z = imu_data[2]
                imu_msg.angular_velocity.x = imu_data[3]
                imu_msg.angular_velocity.y = imu_data[4]
                imu_msg.angular_velocity.z = imu_data[5]

                self.imu_publisher.publish(imu_msg)

                # Publish Ultrasonic Cluster 1
                us_cluster_1_msg = Float32MultiArray()
                us_cluster_1_msg.data = us1_data
                self.ultrasonic_cluster_1_publisher.publish(us_cluster_1_msg)

                # Publish Ultrasonic Cluster 2
                us_cluster_2_msg = Float32MultiArray()
                us_cluster_2_msg.data = us2_data
                self.ultrasonic_cluster_2_publisher.publish(us_cluster_2_msg)

            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
