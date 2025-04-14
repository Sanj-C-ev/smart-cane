import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import serial
import time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float32MultiArray, Bool

class ESP32SensorNode(Node):
    def __init__(self):
        super().__init__('esp32_sensor_node')
        
        # QoS Profiles
        self._qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self._qos_critical = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB1'),
                ('baud_rate', 115200),
                ('max_retries', 5),
                ('retry_delay', 1.0),
                ('publish_rate', 20.0),  # Hz
            ]
        )
        
        # Publishers (with QoS)
        self._imu_pub = self.create_publisher(
            Imu, 'sensors/imu', self._qos_critical)
        self._us1_pub = self.create_publisher(
            Float32MultiArray, 'sensors/ultrasonic_cluster_1', self._qos_sensor)
        self._us2_pub = self.create_publisher(
            Float32MultiArray, 'sensors/ultrasonic_cluster_2', self._qos_sensor)
        self._switch_pub = self.create_publisher(
            Bool, 'sensors/switch', self._qos_sensor)
        self._gps_pub = self.create_publisher(
            NavSatFix, 'sensors/gps', self._qos_sensor)
        
        # Serial connection
        self._serial = None
        self._connect_serial()
        
        # Timer (dynamic rate from parameters)
        self._timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.read_serial_data
        )
        self.get_logger().info("Sensor node ready")

    def _connect_serial(self):
        """Establish serial connection with retries."""
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        max_retries = self.get_parameter('max_retries').value
        retry_delay = self.get_parameter('retry_delay').value
        
        for attempt in range(max_retries):
            try:
                self._serial = serial.Serial(
                    port=port,
                    baudrate=baud,
                    timeout=0.1,
                    write_timeout=1,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                self._serial.reset_input_buffer()
                self.get_logger().info(f"Connected to {port} at {baud} baud")
                return
            except serial.SerialException as e:
                self.get_logger().warn(
                    f"Attempt {attempt + 1}/{max_retries}: {str(e)}")
                time.sleep(retry_delay * (attempt + 1))  # Exponential backoff
        
        self.get_logger().error("Failed to connect to serial port!")
        raise RuntimeError("Serial connection failed")

    def read_serial_data(self):
        if not self._serial or not self._serial.is_open:
            self._connect_serial()
            return
        
        try:
            while self._serial.in_waiting > 0:
                line = self._serial.readline().decode('utf-8').strip()
                if line:
                    self._process_data_line(line)
        except (UnicodeDecodeError, serial.SerialException) as e:
            self.get_logger().error(f"Serial error: {e}. Reconnecting...")
            self._serial.close()
            self._serial = None

    def _process_data_line(self, line):
        """Process and validate sensor data."""
        try:
            parts = line.split(',')
            data = {
                'BTN': None,
                'IMU': None,
                'US1': None,
                'US2': None,
                'GPS': None
            }
            
            i = 0
            while i < len(parts):
                key = parts[i]
                if key in data:
                    if key == "BTN":
                        data[key] = parts[i+1] == "ON"
                        i += 2
                    elif key == "IMU":
                        data[key] = [float(x) for x in parts[i+1:i+7]]
                        i += 7
                    elif key == "US1":
                        data[key] = [float(x) for x in parts[i+1:i+4]]
                        i += 4
                    elif key == "US2":
                        data[key] = [float(parts[i+1])]
                        i += 2
                    elif key == "GPS":
                        data[key] = (parts[i+1], parts[i+2])
                        i += 3
                else:
                    i += 1
            
            # Publish validated data
            if data['BTN'] is not None:
                self._switch_pub.publish(Bool(data=data['BTN']))
            
            if data['IMU'] and len(data['IMU']) == 6:
                self._publish_imu(data['IMU'])
            
            if data['US1']:
                self._us1_pub.publish(Float32MultiArray(data=data['US1']))
            
            if data['US2']:
                self._us2_pub.publish(Float32MultiArray(data=data['US2']))
            
            if data['GPS']:
                self._publish_gps(*data['GPS'])
                
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Invalid data: {line} ({e})")

    def _publish_imu(self, values):
        """Publish IMU data with validation."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = values[0]
        msg.linear_acceleration.y = values[1]
        msg.linear_acceleration.z = values[2]
        msg.angular_velocity.x = values[3]
        msg.angular_velocity.y = values[4]
        msg.angular_velocity.z = values[5]
        self._imu_pub.publish(msg)

    def _publish_gps(self, lat_str, lon_str):
        """Convert and publish GPS data."""
        try:
            lat = self._dms_to_decimal(lat_str)
            lon = self._dms_to_decimal(lon_str)
            if abs(lat) > 90 or abs(lon) > 180:
                raise ValueError("Invalid GPS coordinates")
            
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            msg.latitude = lat
            msg.longitude = lon
            msg.status.status = NavSatStatus.STATUS_FIX
            self._gps_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"GPS error: {e}")

    @staticmethod
    def _dms_to_decimal(dms_str):
        """Convert DMS to decimal degrees."""
        if dms_str == "0.0":
            return 0.0
            
        parts = dms_str.replace('°', ' ').replace('\'', ' ').replace('"', ' ').split()
        degrees = float(parts[0])
        minutes = float(parts[1])
        seconds = float(parts[2])
        direction = parts[3]
        
        decimal = degrees + (minutes / 60.0) + (seconds / 3600.0)
        return -decimal if direction in ['S', 'W'] else decimal

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()