#!/usr/bin/env python3

import socket
import numpy as np

# HOST = 'localhost'  # The server's hostname or IP address
HOST = '192.168.43.232'  # The server's hostname or IP address
PORT = 8080        # The port used by the server


m_height = 10
m_width = 10
m_size = m_height*m_width
buffer_size = 100
# matrix = []

print(f'height = {m_height}; width = {m_width}')
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    # s.sendall(b'Hello, world')
    nrecv = 0

    while True:
        i = 0
        cur_buf_size = buffer_size
        data = bytes()
        while i < m_size:
            cur_buf_size = min(m_size-i, buffer_size)
            data =  data + (s.recv(cur_buf_size))
            i+=cur_buf_size

        matrix = np.reshape(np.frombuffer(data, dtype=np.uint8), newshape=(m_height, m_width))
        nrecv+=1
        print(nrecv)
        print("Received:")
        print(matrix)
    # for i in range(height):
    #     new = []
    #     for j in range(width):
    #         new.append(s.recv(4));
    #     matrix.append(new)

    # data = s.recv(1024)

# print('Received', repr(data))


# for i in range(len(matrix)):
#     print(matrix[i])
