#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from my_msgs.msg import Path

import socket
import numpy as np
import time

# HOST = 'localhost'  # The server's hostname or IP address
HOST = '192.168.10.10'  # The server's hostname or IP address
PORT = 8080        # The port used by the server


m_height = 100
m_width = 100
m_size = m_height*m_width
buffer_size = 10000
# buffer_size = 32768


class PathPub(Node):

    def __init__(self):
        super().__init__('path_maker')
        self.get_logger().info('Started /path_pub node')

        self.path_publisher_ = self.create_publisher(Path, 'path', 10)
        
        self.get_logger().info('Started publisher on /path topic')
        self.get_logger().info(f'image: height = {m_height}; width = {m_width}')
        
        self.msg_counter = 0
        self.path_msg = Path()
        
    def receive(self):
        total_time = 0.0
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))

            while True:
                i = 0
                cur_buf_size = buffer_size
                data = bytes()
                prev_data_len = 0
                cur_length = 0
                begin = 0.0
                end = 0.0
                while i < m_size:
                    begin = time.perf_counter_ns()
                    cur_buf_size = min(m_size-i, buffer_size)
                    
                    data =  data + (s.recv(cur_buf_size))
                    cur_length = len(data)
                    if prev_data_len == cur_length:
                        self.get_logger().info('Empty Package -> Ending')
                        return total_time
                    prev_data_len = cur_length
                    print(cur_length)
                    i = cur_length
                
                end = time.perf_counter_ns()
                total_time += (end-begin)
                matrix = np.reshape(np.frombuffer(data, dtype=np.uint8), newshape=(m_height, m_width))
                
                # ==========================================================
                print(self.msg_counter)
                print("Received:")
                print(matrix[0][0])
                print(matrix[0][1])
                print(matrix[1][0])
                
                path_x = list()
                path_y = list()
                for i in range(4):
                    path_x.append(i + 0.5*self.msg_counter)
                    path_y.append(i + 0.25*self.msg_counter)
                # ==========================================================
                    
                self.path_msg.x = path_x
                self.path_msg.y = path_y
                self.path_msg.header.stamp = self.get_clock().now().to_msg()
                self.path_msg.header.frame_id = "path_msg_{:07d}".format(self.msg_counter)
                self.msg_counter += 1
                
                self.path_publisher_.publish(self.path_msg)
                

def main(args=None):
    
    rclpy.init(args=args)
    node = PathPub()
    total_time = node.receive()
    node.get_logger().info(f'total_time: {total_time/10**6}; average  = {total_time/(node.msg_counter*10**6)}')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()