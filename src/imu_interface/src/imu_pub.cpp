#include <stdio.h>
#include <string.h>
#include <math.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <bcm2835.h>
#include "../include/imu_interface/I2Cdev.h"
#include "../include/imu_interface/MPU6050.h"
#include "../include/imu_interface/MPU6050_6Axis_MotionApps_V6_12.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define ROS_TIME_NOW()										__global_clock_->now()

MPU6050 mpu;

// #define OUTPUT_READABLE_YAWPITCHROLL
// #define OUTPUT_READABLE_REALACCEL
// #define OUTPUT_READABLE_WORLDACCEL
// #define OUTPUT_READABLE_CUSTOM


bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

class ImuPub : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr __global_clock_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  long long int msg_counter = 0;
  char str_buffer[500];
// printf("\n");
  public:
    ImuPub()
    : Node("imu_pub"){
        printf("IMU_PUB constructor\n");
        __global_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
        MPU6050::do_nothing();
        I2Cdev::initialize();
        delay(5000); // So you have time to launch the serial monitor
        // RCLCPP_INFO(this->get_logger(), "Hello23: \n");
        mpu.initialize();
        // RCLCPP_INFO(this->get_logger(), "Hello24: \n");
        devStatus = mpu.dmpInitialize();
        // RCLCPP_INFO(this->get_logger(), "Hello3: \n");
        printf("devStatus = %d\n", devStatus);
        if (devStatus == 0) {
            mpu.setDMPEnabled(true);                // turn on the DMP, now that it's ready
            mpuIntStatus = mpu.getIntStatus();
            printf("mpuIntStatus = %d\n", mpuIntStatus);
            dmpReady = true;                        // set our DMP Ready flag so the main loop() function knows it's okay to use it
            packetSize = mpu.dmpGetFIFOPacketSize();      // get expected DMP packet size for later comparison
            printf("packetSize = %d\n", packetSize);
        } else {                                          // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
            printf("DMP Initialization failed (code %d)", devStatus);
            delay(2000);
        }
        // RCLCPP_INFO(this->get_logger(), "Hello4: \n");
        using namespace std::chrono_literals;
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        timer_ = this->create_wall_timer(
        20ms, std::bind(&ImuPub::timer_callback, this));
        // RCLCPP_INFO(this->get_logger(), "Hello5: \n");
        printf("IMU_PUB constructor END\n");
    }
    
  private:
    void timer_callback()
    {
      printf("TIMER CALLBACK\n");
      auto imu_msg = sensor_msgs::msg::Imu();
      
      if (!dmpReady);                                                    // if programming failed, don't try to do anything
      mpuInterrupt = true;
      fifoCount = mpu.getFIFOCount();                                           // get current FIFO count
      printf("mpuIntStatus = %d\n", mpuIntStatus);
      printf("first fifoCount = %d\n", fifoCount);
      if ((mpuIntStatus & 0x10) || fifoCount == 1024)                           // check for overflow (this should never happen unless our code is too inefficient)
      {
          mpu.resetFIFO();                                                      // reset so we can continue cleanly
          printf("FIFO overflow!");
      } 
      else if (mpuIntStatus & 0x01)                                             // otherwise, check for DMP data ready interrupt (this should happen frequently)
      {    
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();        // wait for correct available data length, should be a VERY short wait
          printf("second fifoCount = %d\n", fifoCount);
          mpu.getFIFOBytes(fifoBuffer, packetSize);                             // read a packet from FIFO
          fifoCount -= packetSize;                                              // track FIFO count here in case there is > 1 packet available
          printf("third fifoCount = %d\n", fifoCount);
        //   #ifdef OUTPUT_READABLE_YAWPITCHROLL                                               // display Euler angles in degrees
        //   mpu.dmpGetQuaternion(&q, fifoBuffer);
        //   mpu.dmpGetGravity(&gravity, &q);
        //   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        //   yaw = ypr[0] * 180 / PI;
        //   pitch = ypr[1] * 180 / PI;
        //   roll = ypr[2] * 180 / PI;
        //   printf("ypr: %f,\t %f,\t %f\n", yaw, pitch, roll);
        //   #endif
        //   #ifdef OUTPUT_READABLE_REALACCEL
              // display real acceleration, adjusted to remove gravity
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          imu_msg.orientation.w = q.w;
          imu_msg.orientation.x = q.x;
          imu_msg.orientation.y = q.y;
          imu_msg.orientation.z = q.z;
          printf("W: %f\t X: %f\t Y: %f\t Z: %f\n", q.w, q.x, q.y, q.z);
          
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          yaw = ypr[0] * 180 / PI;
          pitch = ypr[1] * 180 / PI;
          roll = ypr[2] * 180 / PI;
          printf("ypr: %f,\t %f,\t %f\n", yaw, pitch, roll);
          
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          imu_msg.linear_acceleration.x = aaReal.x;
          imu_msg.linear_acceleration.y = aaReal.y;
          imu_msg.linear_acceleration.z = aaReal.z;
          printf("areal: %d,\t %d,\t %d\n", aaReal.x, aaReal.y, aaReal.z);
        //   #endif
        //   #ifdef OUTPUT_READABLE_WORLDACCEL
        //       // display initial world-frame acceleration, adjusted to remove gravity
        //       // and rotated based on known orientation from quaternion
        //       mpu.dmpGetQuaternion(&q, fifoBuffer);
        //       mpu.dmpGetAccel(&aa, fifoBuffer);
        //       mpu.dmpGetGravity(&gravity, &q);
        //       mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        //       mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        //       printf("aworld: %d,\t %d,\t %d\n", aaWorld.x, aaWorld.y, aaWorld.z);
        //   #endif
        //   #ifdef OUTPUT_READABLE_CUSTOM
            //   mpu.dmpGetQuaternion(&q, fifoBuffer);
            //   printf("W: %f\t X: %f\t Y: %f\t Z: %f\n", q.w, q.x, q.y, q.z);
        //   #endif
          mpu.dmpGetGyro(&gy, fifoBuffer);
          printf("gyro: %d,\t %d,\t %d\n", gy.x, gy.y, gy.z);
          imu_msg.angular_velocity.x = gy.x;
          imu_msg.angular_velocity.y = gy.y;
          imu_msg.angular_velocity.z = gy.z;
          
          RCLCPP_INFO(this->get_logger(), "Publishing: \n");
          imu_msg.header.stamp = ROS_TIME_NOW();
          int string_len; 
          string_len = sprintf(str_buffer, "imu_msg_%07lld", msg_counter++);
          imu_msg.header.frame_id = std::string(str_buffer);
          
          publisher_->publish(imu_msg);
          printf("TIMER CALLBACK END\n");
      }else{
          printf("uhhhhhhh\n");
      }
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPub>());
  rclcpp::shutdown();
  return 0;
}