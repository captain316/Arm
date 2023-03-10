#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <errno.h>
#include <iostream>
#include <odom_pub/motor_msg.h>

#define RECV_LEN 50

#define SERV_PORT 8000

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_publisher");

    ros::NodeHandle n;
    ros::Publisher encoder_pub = n.advertise<odom_pub::motor_msg>("encoder_msg", 500);
    odom_pub::motor_msg encoder;

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    int recv_num;
    if(sock_fd < 0) {  
        perror("socket");  
        exit(1);  
    }  
    struct sockaddr_in addr_serv;
    memset(&addr_serv, 0, sizeof(struct sockaddr_in));
    addr_serv.sin_family = AF_INET;
    addr_serv.sin_port = htons(SERV_PORT); 
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY); // 无论哪个网卡，只要是目标的端口，就能收到数据
    int len = sizeof(addr_serv);
    if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0) { 
        perror("bind error!!!!");  
        exit(1);
    }
    // 20 可以调
    char recv_buf[RECV_LEN];
    
    // struct sockaddr_in addr_client;

    while(ros::ok()) { 
        recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_serv, (socklen_t *)&len);  
        if(recv_num < 0) {   
            perror("recvfrom error:");  
            exit(1);       
        }
        //std::cout << "data is " << recv_buf << std::endl;
        std::string data1 = "", data2 = "";
        int t = 0;
        bool flag = false;

        // 协议解析
        for(int i = 0; i < RECV_LEN; i++) {
            if(!flag) {
                if(recv_buf[i] == ',') {    //用逗号分开的
                    t = i + 1;
                    flag = true;
                }
                else
                    data1 += recv_buf[i];
            }
            else {            
                if(('9' < recv_buf[i] || recv_buf[i] < '0') && recv_buf[i] != '.')
                    break;
                data2 += recv_buf[i];
            }
        }  

        // 不知道数据是怎么发布，先写两个
        encoder.driver_motor_01 = std::stod(data1);
        encoder.steer_motor_01 = std::stod(data2);
        std::cout << "encoder.driver_motor_01 = " << encoder.driver_motor_01 << "   encoder.steer_motor_01 = " << encoder.steer_motor_01 << std::endl;
        encoder_pub.publish(encoder);

    }
    close(sock_fd);
    return 0;
    
}
