#!/usr/bin/env python3
import rclpy
import pynmea2
import io
import serial
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix 

class PubSub(Node):

    def __init__(self):
        super().__init__('arduino_interface')
        self.get_logger().info('Started /gps_interface node')

        self.servo_publisher_ = self.create_publisher(NavSatFix, 'fix', 10)
        self.get_logger().info('Started publisher on /fix topic')

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.gps_callback)

        self.ser = serial.Serial('/dev/ttyS0', 9600, timeout=5.0)
        self.sio = io.TextIOWrapper(io.BufferedRWPair(self.ser, self.ser))

        
    def gps_callback(self):
            gps_msg = NavSatFix()
            line = self.sio.readline()
            msg = pynmea2.parse(line)
            gps_msg.latitude = msg.latitude
            gps_msg.longitude = msg.longitude
            gps_msg.altitude = msg.altitude


def main(args=None):
    
    rclpy.init(args=args)
    node = PubSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()