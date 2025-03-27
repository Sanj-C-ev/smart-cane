import rclpy
import serial
import time
from rclpy.node import Node
from std_msgs.msg import String

class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')
        
        # Create publisher for ultrasonic data
        self.publisher_ = self.create_publisher(String, 'ultrasonic_data', 10)

        # Try to connect to the serial port
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Change to match your port
            self.get_logger().info("Connected to serial port: /dev/ttyUSB0")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            exit(1)

        # Create a timer to read serial data every 100ms
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        """Reads data from the serial port and publishes it to the ROS topic."""
        try:
            if self.serial_port.in_waiting:
                data = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                
                # Validate data format (expected: "50,30,40")
                if self.validate_data(data):
                    msg = String()
                    msg.data = data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published: {data}")
                else:
                    self.get_logger().warn(f"Invalid data received: {data}")

        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")

    def validate_data(self, data):
        """Checks if the incoming data is in the expected format (e.g., '50,30,40')."""
        try:
            values = list(map(float, data.split(',')))
            return len(values) == 3  # Expecting three values (front, left, right)
        except ValueError:
            return False  # Data is invalid

def main(args=None):
    rclpy.init(args=args)
    node = SerialReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
