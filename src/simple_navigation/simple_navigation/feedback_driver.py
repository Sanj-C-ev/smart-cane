#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import serial
import time


class HapticBridgeNode(Node):
    def __init__(self):
        super().__init__('haptic_bridge_node')
        
        # Configuration parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('http_ip', '192.168.9.178'),
                ('http_timeout', 1.5),
                ('http_retries', 3),
                ('serial_port', '/dev/ttyUSB1'),
                ('serial_baud', 115200),
                ('serial_timeout', 1.0)
            ])
        
        # HTTP configuration for haptic feedback
        self.esp32_http_url = f"http://{self.get_parameter('http_ip').value}/control"
        self.http_timeout = self.get_parameter('http_timeout').value
        self.http_retries = self.get_parameter('http_retries').value
        
        # Serial configuration for kinesthetic feedback
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('serial_baud').value
        self.serial_timeout = self.get_parameter('serial_timeout').value
        self.serial_connection = None
        
        # Initialize subscribers with QoS profiles
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.haptic_sub = self.create_subscription(String, '/haptic_feedback', self.haptic_callback, qos_profile)
        
        self.kinesthetic_sub = self.create_subscription(
            String,
            '/kinesthetic_feedback',
            self.kinesthetic_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        
        # Initialize connections
        self.init_serial()
        self.get_logger().info("Haptic Bridge Node Initialized - Ready to forward commands")

    def init_serial(self):
        """Initialize and maintain serial connection"""
        try:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()
                
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout
            )
            time.sleep(2)  # Allow time for connection to stabilize
            self.get_logger().info(f"Serial connection established on {self.serial_port}")
            return True
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {str(e)}")
            self.serial_connection = None
            return False

    def validate_haptic_command(self, command_str):
        """Validate the haptic feedback command format"""
        parts = command_str.split('&')
        if len(parts) != 4:
            raise ValueError("Expected 4 motor commands (M1-M4)")
        
        for i, part in enumerate(parts):
            if not part.startswith(f'M{i+1}='):
                raise ValueError(f"Invalid motor command format at position {i}")
            
            try:
                value = int(part.split('=')[1])
                if not 0 <= value <= 255:
                    raise ValueError(f"Motor value out of range (0-255) in {part}")
            except (IndexError, ValueError):
                raise ValueError(f"Invalid motor value in {part}")

    def send_haptic_command(self, command_str):
        """Send haptic command with retry logic"""
        last_error = None
        
        for attempt in range(self.http_retries):
            try:
                response = requests.post(
                    self.esp32_http_url,
                    data=command_str,
                    timeout=self.http_timeout,
                    headers={
                        'Content-Type': 'application/x-www-form-urlencoded',
                        'Connection': 'close'
                    }
                )
                
                if response.status_code == 200:
                    return True, response.text.strip()
                
                self.get_logger().warning(
                    f"Attempt {attempt+1}: HTTP {response.status_code} - {response.text.strip()}")
                
            except requests.exceptions.RequestException as e:
                last_error = str(e)
                self.get_logger().warning(f"Attempt {attempt+1}: HTTP failed - {last_error}")
                continue
                
        return False, last_error or "Unknown error"

    def haptic_callback(self, msg):
        """Process haptic feedback commands"""
        if not msg.data.strip():
            self.get_logger().debug("Received empty haptic command")
            return
            
        try:
            # Validate command format
            self.validate_haptic_command(msg.data)
            
            # Send with retry logic
            success, response = self.send_haptic_command(msg.data)
            
            if success:
                self.get_logger().debug(f"Haptic command successful: {response}")
            else:
                self.get_logger().error(f"Haptic command failed: {response}")
                
        except ValueError as ve:
            self.get_logger().error(f"Invalid haptic command: {str(ve)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected haptic error: {str(e)}")

    def kinesthetic_callback(self, msg):
        """Process kinesthetic feedback commands"""
        if not msg.data.strip():
            self.get_logger().debug("Received empty kinesthetic command")
            return
            
        command_str = msg.data.strip()
        if not command_str.endswith('\n'):
            command_str += '\n'
        
        self.get_logger().debug(f"Sending kinesthetic command: {command_str.strip()}")
        
        # Ensure serial connection is active
        if not (self.serial_connection and self.serial_connection.is_open):
            if not self.init_serial():
                self.get_logger().error("Cannot send - serial connection unavailable")
                return
                
        try:
            self.serial_connection.write(command_str.encode())
            self.get_logger().debug("Kinesthetic command sent successfully")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {str(e)}")
            self.init_serial()  # Attempt reconnection

    def __del__(self):
        """Cleanup resources"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info("Serial connection closed")

def main():
    rclpy.init()
    node = HapticBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    except Exception as e:
        node.get_logger().fatal(f"Fatal error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()