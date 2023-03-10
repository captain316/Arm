#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/bind.hpp>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <json/json.h>
#include "std_msgs/Int32MultiArray.h"
#include <vector>
#define PI  3.14159
#define W   0.386
#define L   0.520
#define R   0.3238
#define DEST_PORT 8005
#define DEST_IP_ADDRESS "192.168.1.103"
#define cos_theta 0.59605
#define sin_theta 0.80296
ros::Publisher codingMsgPub;
// 全向消息
class omni_msg {
    public:
        omni_msg() {
            
            wheel_1_v = 0.;
            wheel_2_v = 0.;
            wheel_3_v = 0.;
            wheel_4_v = 0.;
            
            wheel_1_angle = 0.;
            wheel_2_angle = 0.;
            wheel_3_angle = 0.;
            wheel_4_angle = 0.;
        }
        ~omni_msg() {}
        double wheel_1_v, wheel_2_v, wheel_3_v, wheel_4_v;
        double wheel_1_angle, wheel_2_angle, wheel_3_angle, wheel_4_angle;
};



// socket通信相应的参数
struct socket_parameters
{
    int sock_fd;    // socket文件描述符
    struct sockaddr_in addr_serv;
    socklen_t len;
    double send_buf[8] = {0., 0., 0., 0., 0., 0., 0., 0.};
};


std::string DataToJson(double vel2, double vel4, double vel3, double vel1, 
                        double angle2, double angle4, double angle3, double angle1)
{
	Json::FastWriter write;
	Json::Value writevalue;
	writevalue["vel2"] = vel2;
	writevalue["vel4"] = vel4;
    writevalue["vel3"] = vel3;
    writevalue["vel1"] = vel1;
    writevalue["angle2"] = angle2;
    writevalue["angle4"] = angle4;
    writevalue["angle3"] = angle3;
    writevalue["angle1"] = angle1;
	std::string data = write.write(writevalue);
	return data; 
}


ros::Subscriber sub;

void cmd_vel_msg_listener_call_back(const boost::shared_ptr<geometry_msgs::Twist const> msg, socket_parameters& socket_param)
{
    std::cout << "recived /cmd_vel topic-------------------" << std::endl;

    double r, dis, angle = 0;
    int flag = 0;
    double v = 0;
    omni_msg send_msg;
    /*
        x、y、theta
        flag:   0 原地旋转
                1 横行
                2 斜行
                3 直行、转弯
    */
    
    // 原地旋转
    if (abs(msg->linear.x) <= 0.01 && abs(msg->linear.y) <= 0.01 && abs(msg->angular.z) >= 0.02) {
        std::cout << "-----------------自转命令---------------------" << std::endl;
        flag = 0;
        v = (-1) * R * msg->angular.z;  // 向左旋转为正
        send_msg.wheel_1_v = v;
        send_msg.wheel_2_v = -v;
        send_msg.wheel_3_v = v;
        send_msg.wheel_4_v = -v;
        send_msg.wheel_1_angle = -53.4132;
        send_msg.wheel_2_angle = 53.4132;
        send_msg.wheel_3_angle = 53.4132;
        send_msg.wheel_4_angle = -53.4132;
        std::cout << "-----------------v = " << (-1) * v << "---------------------" << std::endl;
    } 
    else if(abs(msg->linear.x) <= 0.01 && abs(msg->linear.y) <= 0.01 && abs(msg->angular.z) <= 0.02) {
        send_msg.wheel_1_v = 0.0;
        send_msg.wheel_2_v = 0.0;
        send_msg.wheel_3_v = 0.0;
        send_msg.wheel_4_v = 0.0;
        send_msg.wheel_1_angle = 0.0;
        send_msg.wheel_2_angle = 0.0;
        send_msg.wheel_3_angle = 0.0;
        send_msg.wheel_4_angle = 0.0;
    }
    // 横行
    else if(abs(msg->linear.x) <= 0.01 && abs(msg->linear.y) >= 0.01 && abs(msg->angular.z) <= 0.02) {
        std::cout << "-----------------横行命令---------------------" << std::endl;
        flag = 1;
        v = msg->linear.y;
        send_msg.wheel_1_v = v;
        send_msg.wheel_2_v = v;
        send_msg.wheel_3_v = v;
        send_msg.wheel_4_v = v;
        send_msg.wheel_1_angle = 90.0;
        send_msg.wheel_2_angle = 90.0;
        send_msg.wheel_3_angle = 90.0;
        send_msg.wheel_4_angle = 90.0;
        std::cout << "-----------------v = " << v << "---------------------" << std::endl;
    } 
    // 斜行
    else if(abs(msg->linear.x) >= 0.01 && abs(msg->linear.y) >= 0.01 && abs(msg->angular.z) <= 0.02) {
        std::cout << "-----------------斜行命令---------------------" << std::endl;
        flag = 2;
        v = sqrt( pow(msg->linear.x, 2 ) + pow(msg->linear.y, 2) );
        double tan_ = msg->linear.y / msg->linear.x;
        angle = atan(tan_) / PI * 180.0;
        if(msg->linear.x < 0) {
            v *= (-1);
            std::cout << "----------------后退命令--------------------" << std::endl;
        } 
        send_msg.wheel_1_v = v;
        send_msg.wheel_2_v = v;
        send_msg.wheel_3_v = v;
        send_msg.wheel_4_v = v;
        send_msg.wheel_1_angle = angle;
        send_msg.wheel_2_angle = angle;
        send_msg.wheel_3_angle = angle;
        send_msg.wheel_4_angle = angle;
        std::cout << "-----------------v = " << v << "---------------------" << std::endl;
    }
    // 直行
    // else if(abs(msg->linear.x) >= 0.01 && abs(msg->linear.y) <= 0.01 && abs(msg->angular.z) <= 0.02) {
    //     flag = 3;
    //     v = msg->linear.x;
    //     std::cout << "-----------------直行命令---------------------" << std::endl;
    //     std::cout << "-----------------v = " << v << "---------------------" << std::endl;
    // } 
    // 停止
    else {
        std::cout << "-----------------直行、转弯命令---------------------" << std::endl;
        flag = 3;
        double omega_vx = msg->angular.z * 0.193;
        double omega_vy = msg->angular.z * 0.260;
        double A = msg->linear.x - omega_vx;
        double B = msg->linear.x + omega_vx;
        double C = msg->linear.y - omega_vy;
        double D = msg->linear.y + omega_vy;
        send_msg.wheel_1_v = sqrt( pow(A, 2) + pow(D, 2) );
        send_msg.wheel_2_v = sqrt( pow(B, 2) + pow(D, 2) );
        send_msg.wheel_3_v = sqrt( pow(A, 2) + pow(C, 2) );
        send_msg.wheel_4_v = sqrt( pow(B, 2) + pow(C, 2) );
        send_msg.wheel_1_angle = atan(D / A) / PI * 180.0;
        send_msg.wheel_2_angle = atan(D / B) / PI * 180.0;
        send_msg.wheel_3_angle = atan(C / A) / PI * 180.0;
        send_msg.wheel_4_angle = atan(C / B) / PI * 180.0;
        if(A < 0) {
            send_msg.wheel_1_v *= (-1);
            send_msg.wheel_3_v *= (-1);
        }
        if(B < 0) {
            send_msg.wheel_2_v *= (-1);
            send_msg.wheel_4_v *= (-1);
        }
    }
    std::cout << "------------------  send_msg.wheel_1_v = " << send_msg.wheel_1_v << "  -------------------" << std::endl;
    std::cout << "------------------  send_msg.wheel_2_v = " << send_msg.wheel_2_v << "  -------------------" << std::endl;
    std::cout << "------------------  send_msg.wheel_3_v = " << send_msg.wheel_3_v << "  -------------------" << std::endl;
    std::cout << "------------------  send_msg.wheel_4_v = " << send_msg.wheel_4_v << "  -------------------" << std::endl;
    std::cout << "------------------  send_msg.wheel_1_angle = " << send_msg.wheel_1_angle << "  -------------------" << std::endl;
    std::cout << "------------------  send_msg.wheel_2_angle = " << send_msg.wheel_2_angle << "  -------------------" << std::endl;
    std::cout << "------------------  send_msg.wheel_3_angle = " << send_msg.wheel_3_angle << "  -------------------" << std::endl;
    std::cout << "------------------  send_msg.wheel_4_angle = " << send_msg.wheel_4_angle << "  -------------------" << std::endl;
    socket_param.send_buf[0] = abs(send_msg.wheel_2_v) < 1.0 ? send_msg.wheel_2_v : 1.0;
    socket_param.send_buf[1] = abs(send_msg.wheel_4_v) < 1.0 ? send_msg.wheel_4_v : 1.0;
    socket_param.send_buf[2] = abs(send_msg.wheel_3_v) < 1.0 ? send_msg.wheel_3_v : 1.0;
    socket_param.send_buf[3] = abs(send_msg.wheel_1_v) < 1.0 ? send_msg.wheel_1_v : 1.0;
    
    // socket_param.send_buf[0] = send_msg.wheel_2_v;
    // socket_param.send_buf[1] = send_msg.wheel_4_v;
    // socket_param.send_buf[2] = send_msg.wheel_3_v;
    // socket_param.send_buf[3] = send_msg.wheel_1_v;
    
    socket_param.send_buf[4] = send_msg.wheel_2_angle;
    socket_param.send_buf[5] = send_msg.wheel_4_angle;
    socket_param.send_buf[6] = send_msg.wheel_3_angle;
    socket_param.send_buf[7] = send_msg.wheel_1_angle;
    

    std::string vParam = DataToJson(socket_param.send_buf[0], socket_param.send_buf[1], socket_param.send_buf[2], socket_param.send_buf[3],
                                    socket_param.send_buf[4], socket_param.send_buf[5], socket_param.send_buf[6], socket_param.send_buf[7]
                                    );
    char arr[1024];
    strcpy(arr, vParam.c_str());

    int send_num;
    // 这里的长度到底是字节数还是数组长度,先按字节数写
    send_num = sendto(socket_param.sock_fd, arr, strlen(arr), 0, (struct sockaddr *)&socket_param.addr_serv, socket_param.len);
    if(send_num < 0) {
        perror("message send error!!!!!!");
        exit(1);
    }
    std::vector<int> coding_msg; 
    
    coding_msg.push_back( int(socket_param.send_buf[0] * (5600) / (0.27463 * PI)) );
    coding_msg.push_back( int(socket_param.send_buf[4] * (450 * 131072) / 360  ) );
    coding_msg.push_back( int(socket_param.send_buf[5] * (450 * 131072) / 360  ) );
    coding_msg.push_back( int(socket_param.send_buf[1] * (5600) / (0.27463 * PI)) );
    coding_msg.push_back( int(socket_param.send_buf[2] * (5600) / (0.27463 * PI)) );
    coding_msg.push_back( int(socket_param.send_buf[6] * (450 * 131072) / 360  ) );
    coding_msg.push_back( int(socket_param.send_buf[7] * (450 * 131072) / 360  ) );
    coding_msg.push_back( int(socket_param.send_buf[3] * (5600) / (0.27463 * PI)) );
    std_msgs::Int32MultiArray code;
    code.data = coding_msg;
    codingMsgPub.publish(code);
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "cmd_vel_pub");
    ros::NodeHandle n;
    
    socket_parameters socket_param;
    
    socket_param.sock_fd = socket(AF_INET, SOCK_DGRAM, 0);   //建立udp socket
    if(socket_param.sock_fd < 0) {
        perror("socket build error!!!!");
        exit(1);
    }
    // 设置address
    memset(&socket_param.addr_serv, 0, sizeof(socket_param.addr_serv));
    socket_param.addr_serv.sin_family = AF_INET;
    // socket_param.addr_serv.sin_addr.s_addr = inet_addr(DEST_IP_ADDRESS);
    socket_param.addr_serv.sin_port = htons(DEST_PORT);
    inet_pton(AF_INET, "192.168.1.100", &socket_param.addr_serv.sin_addr.s_addr);
    socket_param.len = sizeof(socket_param.addr_serv);

    boost::function<void(const boost::shared_ptr<geometry_msgs::Twist const>&)> callback = boost::bind(cmd_vel_msg_listener_call_back, _1, boost::ref(socket_param));
    sub = n.subscribe("/cmd_vel", 500, callback);    //订阅cmd_vel的话题，/cmd_vel
    codingMsgPub = n.advertise<std_msgs::Int32MultiArray>("encoder_msg", 500);
    
    ROS_INFO("ready!");

    // 这里用spin还是spinOnce？
    ros::spin();
    // 关闭套接字
    close(socket_param.sock_fd);
    return 0;

}
