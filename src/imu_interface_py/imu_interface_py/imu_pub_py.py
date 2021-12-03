#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_msgs.msg import Imu
# from sensor_msgs.msg import Imu
import time 
import math

from imu_interface_py.MPU6050 import MPU6050

i2c_bus = 1
device_address = 0x68
x_accel_offset = +542
y_accel_offset = -335
z_accel_offset = +1000
x_gyro_offset = +1000
y_gyro_offset = -5000
z_gyro_offset = -5
imu_msg = Imu()
msg_counter = 0
mpu = MPU6050()
packet_size = 0
mpuIntStatus = 0

class ImuPub(Node):
    
    def __init__(self):
        global mpuIntStatus, packet_size
        
        super().__init__('imu_pub')
        self.get_logger().info('Started /imu_pub node')
        
        
        self.get_logger().info('Initiated MPU6050 Instance')
        # mpu.initialize()
        # devStatus = mpu.dmpInitialize()
        # print("Dev status = {}".format(devStatus))

        # if devStatus == 0:
        #     mpu.setDMPEnabled(True)      
        #     mpuIntStatus = mpu.getIntStatus()
        #     print("mpuIntStatus = {}".format(mpuIntStatus))
            # packet_size = mpu.dmpGetFIFOPacketSize()
        #     print("packet_size = {}".format(packet_size))
        #     # mpu.resetFIFO() # eu add   
        # else:
        #     print("DMP Initialization failed (code {})".format(devStatus))
        #     # delay(2000)
        #     return
        
        packet_size = 28
        # packet_size = 28*2
        print("packet_size = {}".format(packet_size))
        
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.get_logger().info('Started publisher on /imu topic')

        timer_period = 1/50  # seconds
        self.timer = self.create_timer(timer_period, self.imu_callback)
        print("IMU_PUB constructor END")

    def imu_callback(self):
        global mpuIntStatus, packet_size, msg_counter, imu_msg
        mpuIntStatus = 1 #apagar isso
        # mpu.resetFIFO() # eu add                  
        # print("Callback")
        fifoCount = mpu.getFIFOCount();        
        # print("first fifoCount = {}".format(fifoCount))  
        # print("mpuIntStatus = {}".format(mpuIntStatus))
        if (mpuIntStatus & 0x10) or fifoCount == 1024:
            mpu.resetFIFO()
            print("FIFO overflow!")
        elif (mpuIntStatus & 0x01):
            # print("elif - ok")
            # print("packet_size = {}".format(packet_size))
            while (fifoCount < packet_size):
                # print("fifoCount = {}".format(fifoCount))
                fifoCount = mpu.getFIFOCount()
            # print("second fifoCount = {}".format(fifoCount))
            
            fifo_buffer = mpu.getFIFOBytes(packet_size)
            # print(fifo_buffer)
            fifoCount -= packet_size
            # print("third fifoCount = {}".format(fifoCount))
            
            q = mpu.dmpGetQuaternion(fifo_buffer)
            imu_msg.orientation.w = q.w
            imu_msg.orientation.x = q.x
            imu_msg.orientation.y = q.y
            imu_msg.orientation.z = q.z
            # print("W: {}\t X: {}\t Y: {}\t Z: {}".format(q.w, q.x, q.y, q.z))
            
            aa = mpu.dmpGetAccel(fifo_buffer)
            gravity = mpu.dmpGetGravity(q)
            ypr = mpu.dmpGetYawPitchRoll(q, gravity)
            yaw = ypr[0] * 180 / math.pi
            pitch = ypr[1] * 180 / math.pi
            roll = ypr[2] * 180 / math.pi
            imu_msg.ypr.yaw = yaw
            imu_msg.ypr.pitch = pitch
            imu_msg.ypr.roll = roll
            # print("ypr: {},\t {},\t {}".format(yaw, pitch, roll))
            
            aaReal = mpu.dmpGetLinearAccel(aa, gravity)
            # imu_msg.linear_acceleration.x = float(aaReal.x)
            # imu_msg.linear_acceleration.y = float(aaReal.y)
            # imu_msg.linear_acceleration.z = float(aaReal.z)
            imu_msg.linear_acceleration.x = aaReal.x
            imu_msg.linear_acceleration.y = aaReal.y
            imu_msg.linear_acceleration.z = aaReal.z
            # print("areal: {},\t {},\t {}".format(aaReal.x, aaReal.y, aaReal.z))
            
            gy = mpu.dmpGetGyro(fifo_buffer)
            # print("gyro: {},\t {},\t {}".format(gy.x, gy.y, gy.z))
            # imu_msg.angular_velocity.x = float(gy.x)
            # imu_msg.angular_velocity.y = float(gy.y)
            # imu_msg.angular_velocity.z = float(gy.z)
            imu_msg.angular_velocity.x = gy.x
            imu_msg.angular_velocity.y = gy.y
            imu_msg.angular_velocity.z = gy.z
       
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_msg_{:07d}".format(msg_counter)
            msg_counter += 1

            self.publisher_.publish(imu_msg)
        else:
            print("uhhhhhhh")
            # self.get_logger().info('OK {}!'.format(apagar))
            #self.get_logger().info('Publishing: "%f"' %quat.x)
        # print("TIMER CALLBACK END\n")
        
def main(args=None):

    rclpy.init(args=args)
    node = ImuPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


