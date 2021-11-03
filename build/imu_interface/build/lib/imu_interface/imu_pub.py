#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from imu_interface.MPU6050 import MPU6050

class ImuPub(Node):
    i2c_bus = 1
    device_address = 0x68
    # The offsets are different for each device and should be changed
    # accordingly using a calibration procedure
    x_accel_offset = -5489
    y_accel_offset = -1441
    z_accel_offset = 1305
    x_gyro_offset = -2
    y_gyro_offset = -72
    z_gyro_offset = -5
    enable_debug_output = False

    def __init__(self):
        super().__init__('imu_pub')
        self.get_logger().info('Started /imu_pub node')

        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.get_logger().info('Started publisher on /imu topic')

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.imu_callback)
        self.mpu = MPU6050(self.i2c_bus, self.device_address, self.x_accel_offset, self.y_accel_offset,
                    self.z_accel_offset, self.x_gyro_offset, self.y_gyro_offset, self.z_gyro_offset,
                    self.enable_debug_output)
        self.get_logger().info('Initiated MPU6050 Instance')

        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.get_logger().info('Configured MPU6050 Sensor')
        self.imu_msg = Imu()

    def imu_callback(self):
       

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        self.FIFO_buffer = [0]*64
        self.mpu_int_status = self.mpu.get_int_status()
        self.FIFO_count = self.mpu.get_FIFO_count()

        # If overflow is detected by status or fifo count we want to reset
        if (self.FIFO_count == 1024) or (self.mpu_int_status & 0x10):
            self.mpu.reset_FIFO()
            self.get_logger().info('MPU6050 OVERFLOW!')
        # Check if fifo data is ready
        elif (self.mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while self.FIFO_count < self.packet_size:
                self.FIFO_count = self.mpu.get_FIFO_count()
            self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)

            accel = self.mpu.DMP_get_acceleration_int16(self.FIFO_buffer)
            quat = self.mpu.DMP_get_quaternion(self.FIFO_buffer)
            grav = self.mpu.DMP_get_gravity(quat)
            lin = self.mpu.DMP_get_linear_accel(accel, grav)
            vel = self.mpu.get_rotation()

            self.imu_msg.orientation.w = quat.w
            self.imu_msg.orientation.x = quat.x
            self.imu_msg.orientation.y = quat.y
            self.imu_msg.orientation.z = quat.z

            self.imu_msg.angular_velocity.x = float(vel[0])
            self.imu_msg.angular_velocity.x = float(vel[1])
            self.imu_msg.angular_velocity.x = float(vel[2])

            self.imu_msg.linear_acceleration.x = lin.x
            self.imu_msg.linear_acceleration.y = lin.y
            self.imu_msg.linear_acceleration.z = lin.z

            self.publisher_.publish(self.imu_msg)
            self.get_logger().info('Publishing: "%f"' %quat.x)
        
def main(args=None):

    rclpy.init(args=args)
    node = ImuPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


