# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     ld = LaunchDescription()
#     arduino_node = Node(
#         package='arduino_interface',
#         executable='arduino_pub',
#     )

#     encoder_node = Node(
#         package='encoder_interface',
#         executable='encoder_pub',
#     )

#     imu_node = Node(
#         package='imu_interface',
#         executable='imu_pub',
#     )
#     # cam_node = Node(
#     #     package='usb_cam',
#     #     executable='usb_cam_node_exe'
#     # )
#     gps_node = Node(
#         package='gps_interface',
#         executable='gps_pub',
#     )

#     ld.add_action(arduino_node)
#     ld.add_action(encoder_node)
#     ld.add_action(imu_node)
#     ld.add_action(gps_node)

#     return ld
    