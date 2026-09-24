#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class LoRaUSBReaderNode(Node):
    def __init__(self):
        super().__init__('lora_usb_reader_node')
        
        # Parameters for port and baud rate
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        # ROS 2 Publisher
        self.publisher_ = self.create_publisher(String, 'lora/received_message', 10)
        
        # Open serial connection
        self.ser = None
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to Heltec V3 on {port} at {baud} baud.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            
        # Timer to poll serial buffer non-blockingly (100Hz)
        self.timer = self.create_timer(0.01, self.read_serial_callback)

    def read_serial_callback(self):
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Look for the tag sent by your Heltec V3
                    if line.startswith("LORA_DATA:"):
                        payload = line.replace("LORA_DATA:", "", 1)
                        
                        # Publish message to ROS 2
                        msg = String()
                        msg.data = payload
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Published LoRa Data: "{payload}"')
            except Exception as e:
                self.get_logger().warn(f'Error while reading serial buffer: {e}')

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LoRaUSBReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()