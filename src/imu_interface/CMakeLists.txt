# ament_package()
cmake_minimum_required(VERSION 3.8)
project(imu_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(my_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)
# add_definitions(-DROS2 -DROS_DISTRO=${ROS_DISTRO})

ADD_LIBRARY(LibsModule 
    include/${PROJECT_NAME}/I2Cdev.h
    include/${PROJECT_NAME}/I2Cdev.cpp
    include/${PROJECT_NAME}/MPU6050.h
    include/${PROJECT_NAME}/MPU6050.cpp
    include/${PROJECT_NAME}/MPU6050_6Axis_MotionApps_V6_12.h
)

# /usr/local/include/bcm2835.h
link_libraries(LibsModule -lbcm2835 -lm -lcap)

add_executable(imu_pub src/imu_pub.cpp)
# add_executable(imu_pub_copy src/imu_pub_copy.cpp)


target_link_libraries(imu_pub ${catkin_LIBRARIES} bcm2835)
target_link_libraries(imu_pub LibsModule ${catkin_LIBRARIES})

ament_target_dependencies(imu_pub rclcpp std_msgs sensor_msgs)

# target_link_libraries(imu_pub_copy LibsModule ${catkin_LIBRARIES})

# ament_target_dependencies(imu_pub_copy rclcpp std_msgs sensor_msgs)

install(TARGETS
  imu_pub
  DESTINATION lib/${PROJECT_NAME})
  
# install(TARGETS
#   imu_pub_copy 
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
