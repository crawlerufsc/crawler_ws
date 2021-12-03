#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from imu_interface.MPU6050 import MPU6050

# apagar = 0

class ImuPub(Node):
    i2c_bus = 1
    # device_address = 0x68
    # # The offsets are different for each device and should be changed
    # # accordingly using a calibration procedure
    # # x_accel_offset = -5489
    # x_accel_offset = +542
    # # y_accel_offset = -1441
    # y_accel_offset = -335
    # # z_accel_offset = 1305
    # # z_accel_offset = -54
    # z_accel_offset = +542
    # x_gyro_offset = -2
    # y_gyro_offset = -72
    # z_gyro_offset = -5
    # enable_debug_output = False

    def __init__(self):
        self.i2c_bus = 1
        self.device_address = 0x68
        # The offsets are different for each device and should be changed
        # accordingly using a calibration procedure
        # x_accel_offset = -5489
        self.x_accel_offset = +542
        # y_accel_offset = -1441
        self.y_accel_offset = -335
        # z_accel_offset = 1305
        self.z_accel_offset = +1000
        # self.x_gyro_offset = -2
        self.x_gyro_offset = +1000
        self.y_gyro_offset = -5000
        self.z_gyro_offset = -5
        self.enable_debug_output = False
        super().__init__('imu_pub')
        self.get_logger().info('Started /imu_pub node')

        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.get_logger().info('Started publisher on /imu topic')

        # timer_period = 0.001  # seconds
        timer_period = 1/50  # seconds
        self.timer = self.create_timer(timer_period, self.imu_callback)
        
        self.mpu = MPU6050(self.i2c_bus, self.device_address, self.x_accel_offset, self.y_accel_offset,
                    self.z_accel_offset, self.x_gyro_offset, self.y_gyro_offset, self.z_gyro_offset,
                    self.enable_debug_output)
        self.get_logger().info('Initiated MPU6050 Instance')

        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.get_logger().info('Configured MPU6050 Sensor')
        self.imu_msg = Imu()
        
        self.msg_counter = 0


    def imu_callback(self):
       
        
        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        self.FIFO_buffer = [0]*64
        self.mpu_int_status = self.mpu.get_int_status()
        self.FIFO_count = self.mpu.get_FIFO_count()
        # global apagar
        # apagar += 1
        # self.mpu.reset_FIFO()

        # If overflow is detected by status or fifo count we want to reset
        if (self.FIFO_count == 1024) or (self.mpu_int_status & 0x10):
            self.mpu.reset_FIFO()
            # self.get_logger().info('MPU6050 OVERFLOW!')
            # self.get_logger().info('OVER {}!'.format(apagar))
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
            self.imu_msg.angular_velocity.y = float(vel[1])
            self.imu_msg.angular_velocity.z = float(vel[2])

            self.imu_msg.linear_acceleration.x = lin.x
            self.imu_msg.linear_acceleration.y = lin.y
            self.imu_msg.linear_acceleration.z = lin.z
            
            
            # accel = self.mpu.DMP_get_acceleration_int16(self.FIFO_buffer)
            # quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)
            # grav = self.mpu.DMP_get_gravity(quat)
            # roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
            
            # self.imu_msg.linear_acceleration.x = accel.x
            # self.imu_msg.linear_acceleration.y = accel.y
            # self.imu_msg.linear_acceleration.z = accel.z
            
            # self.imu_msg.orientation.w = roll_pitch_yaw.x
            # self.imu_msg.orientation.x = roll_pitch_yaw.y
            # self.imu_msg.orientation.y = roll_pitch_yaw.z
            
            
            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_msg.header.frame_id = "imu_msg_{:07d}".format(self.msg_counter)
            self.msg_counter += 1

            self.publisher_.publish(self.imu_msg)
            # self.get_logger().info('OK {}!'.format(apagar))
            #self.get_logger().info('Publishing: "%f"' %quat.x)
        
def main(args=None):

    rclpy.init(args=args)
    node = ImuPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


