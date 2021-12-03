// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>

#define PORT 8080
#define M_HEIGHT 100
#define M_WIDTH 100
#define M_SIZE M_HEIGHT*M_WIDTH
#define BUFFER_SIZE 10000
// #define BUFFER_SIZE 32768


int main(int argc, char const *argv[])
{
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    // char hello[] =  "Hello from server";

    char matrix[M_HEIGHT][M_WIDTH];

    for (char i = 0; i < M_HEIGHT; ++i) {
      for (char j = 0; j < M_WIDTH; ++j) {
        matrix[i][j] = i*M_WIDTH+j;
      }
    }

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                  &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    // address.sin_addr.s_addr = INADDR_ANY;
    address.sin_addr.s_addr = inet_addr("192.168.10.10");
    address.sin_port = htons( PORT );

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address,
                                 sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                       (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    // valread = read( new_socket , buffer, 1024);
    // printf("%s\n",buffer );
    // send(new_socket, (const void *) M_HEIGHT, sizeof(int), 0);
    // send(new_socket, (const void *) M_WIDTH, sizeof(int), 0);
    int counter = 0;
    while (true) {
      usleep(300000);
      int cur_buf_size = BUFFER_SIZE;
      for (int i = 0; i < M_SIZE;) {

        if (M_SIZE - i < BUFFER_SIZE) {
          cur_buf_size = M_SIZE - i;
        }
        send(new_socket, matrix+i, cur_buf_size, 0);
        i+=BUFFER_SIZE;
      }
      // send(new_socket , matrix, M_HEIGHT*M_WIDTH, 0 );
      counter++;
      printf("%d\n", counter);
      printf("Message sent\n");
    }
    return 0;
}
