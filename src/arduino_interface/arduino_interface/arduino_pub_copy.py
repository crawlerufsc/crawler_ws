#!/usr/bin/env python3
import rclpy
import smbus2 as smbus
from rclpy.node import Node

from sensor_msgs.msg import Joy
from my_msgs.msg import IntH


class PubSub(Node):

    def __init__(self):
        super().__init__('arduino_interface')
        self.get_logger().info('Started /arduino_interface node')

        self.servo_publisher_ = self.create_publisher(IntH, 'servo_state', 10)
        self.get_logger().info('Started publisher on /servo_state topic')

        self.motor_publisher_ = self.create_publisher(IntH, 'motor_state', 10)
        self.get_logger().info('Started publisher on /motor_state topic')

        self.subscription = self.create_subscription(Joy,'/joy', self.listener_callback, 10)
        self.get_logger().info('Started subscriber on /joy topic')

        self.subscription  # prevent unused variable warning

        self.bus = smbus.SMBus(1)
        
        self.msg_counter = 0
        
    def listener_callback(self, joy_msg):
        servo_state_msg = IntH()
        motor_state_msg = IntH()
        
        #self.get_logger().info('Axis 1: %f Axis 3: %f' %(joy_msg.axes[1], joy_msg.axes[3]))
        motor_state_msg.value = int(255*abs((joy_msg.axes[1])))
        servo_state_msg.value = int(45*(joy_msg.axes[3]))+45
        
        motor_state_msg.header.stamp = self.get_clock().now().to_msg()
        servo_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        motor_state_msg.header.frame_id = "motor_msg_{:07d}".format(self.msg_counter)
        servo_state_msg.header.frame_id = "servo_msg_{:07d}".format(self.msg_counter)
        self.msg_counter += 1
        
        self.servo_publisher_.publish(servo_state_msg)
        self.motor_publisher_.publish(motor_state_msg)

        self.bus.write_i2c_block_data(0x0b, 0x00, [motor_state_msg.value, servo_state_msg.value])
        # self.get_logger().info('Servo angle: %d Motor Acceleration: %d' %(servo_state_msg.value, motor_state_msg.value))


def main(args=None):
    
    rclpy.init(args=args)
    node = PubSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()