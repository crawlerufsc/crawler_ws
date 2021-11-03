#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from encoder_interface.AS5600 import AS5600



class OdomPub(Node):
    lastOutput = 0
    revolutions = 0

    def __init__(self):
        super().__init__('encoder_interface')
        self.get_logger().info('Started /encoder_interface node')

        self.publisher_ = self.create_publisher(Pose2D, 'odom', 10)
        self.get_logger().info('Started publisher on /odom topic')

        timer_period = 0.0001  # seconds
        self.timer = self.create_timer(timer_period, self.odom_callback)
        
        self.encoder = AS5600()
        self.get_logger().info('Started AS5600 Encoder')
        

    def odom_callback(self):
        
        odom_msg = Pose2D()
        output = self.encoder.getPosition()

        if ((self.lastOutput - output) > 2047):
            self.revolutions += 1

        if ((self.lastOutput - output) < -2047):
            self.revolutions -= 1

        self.lastOutput = output
        position = self.revolutions*4096 + output

        #odom_msg.x = position * 360/ 4094; #conversion from ecoder ticks to degrees;
        #odom_msg.x = ((position/4094)/48)*2*math.pi*90; #calculates wheel displacement in mm
        odom_msg.x = ((position/4094)/48); #calculates wheel displacement in revolution

        #self.get_logger().info('Encoder position: %f' %odom_msg.x)
        self.publisher_.publish(odom_msg)

def main(args=None):
    
    rclpy.init(args=args)
    node = OdomPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()