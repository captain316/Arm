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


#define PI  3.14159
#define W   0.386
#define L   0.520
#define R   0.3238
#define DEST_PORT 8005
#define DEST_IP_ADDRESS "192.168.1.103"

class cmd_vel_msg
{
    public:
        cmd_vel_msg();
        ~cmd_vel_msg() {};
        double vel_linear_x;
        double vel_linear_y;
        double vel_linear_z;
        double vel_angular_x;
        double vel_angular_y;
        double vel_angular_z;
        double pub_frequency;   // hz

        double angular;         // 累计角度值
};

cmd_vel_msg::cmd_vel_msg()
{
    vel_linear_x = 0.;
    vel_linear_y = 0.;
    vel_linear_z = 0.;
    vel_angular_x = 0.;
    vel_angular_y = 0.;
    vel_angular_z = 0.;
    angular = 0;
    pub_frequency = 10; // 10hz,move_base中cmd_vel发送一次的速率,这个在p3dx_description/config/planner/.yaml中的controller_frequency设置
}

// socket通信相应的参数
struct socket_parameters
{
    int sock_fd;    // socket文件描述符
    struct sockaddr_in addr_serv;
    socklen_t len;
    double send_buf[3] = {0., 0., 0.};
};


std::string DataToJson(double linearV, double angleV, double flag)
{
	Json::FastWriter write;
	Json::Value writevalue;
	writevalue["linearV"] = linearV;
	writevalue["angleV"] = angleV;
    writevalue["flag"] = flag;
	std::string data = write.write(writevalue);
	return data; 
}


ros::Subscriber sub;

void cmd_vel_msg_listener_call_back(const boost::shared_ptr<geometry_msgs::Twist const> msg, cmd_vel_msg& cv_msg, socket_parameters& socket_param)
{
    std::cout << "recived /cmd_vel topic-------------------" << std::endl;
    // c:cmd, v:vel
    cv_msg.vel_linear_x = msg->linear.x;
    cv_msg.vel_linear_y = msg->linear.y;
    cv_msg.vel_linear_z = msg->linear.z;
    cv_msg.vel_angular_x = msg->angular.x;
    cv_msg.vel_angular_y = msg->angular.y;
    cv_msg.vel_angular_z = msg->angular.z;

    double r, dis, angle;
    int flag = 0;
    double v = 0;
    //if(msg->angular.z == 0) {
    //     angle = 0.;
    // } else if (msg->angular.z > 0) {
    //     r = msg->linear.x / msg->angular.z;
    //     dis = r - W / 2;
    //     angle = atan((L / 2) / dis);
    //     if(angle > (PI / 6)) {
    //         angle = (PI / 6);
    //     }
    // } else {
    //     r = msg->linear.x / msg->angular.z;
    //     dis = r + W / 2;
    //     angle = atan((L / 2) / dis);
    //     if(angle < -(PI / 6)) {
    //         angle = -(PI / 6);
    //     }
    // }

    if (abs(msg->angular.z) <= 0.01) {
        flag = 0;
        angle = 0.;
    } 
    else if ( abs(msg->angular.z) > 0.01 && ( abs(msg->linear.x) <= 0.08 ) ) {
        flag = 1;
        v = (-1) * R * msg->angular.z;  // 向左旋转为正
        std::cout << "-----------------自转命令---------------------" << std::endl;
        std::cout << "-----------------v = " << v << "---------------------" << std::endl;
    } 
    else {
        flag = 0;
        if(msg->angular.z > 0) {
            r = msg->linear.x / msg->angular.z;
            if(msg->linear.x >= 0) {
                std::cout << "-----------------左转前进---------------------" << std::endl;
                dis = r - W / 2;
                angle = atan((L / 2) / dis);
            } else {
                dis = r + W / 2;
                angle = atan((L / 2) / dis);
            }
        } else {
            r = msg->linear.x / msg->angular.z;
            if(msg->linear.x >= 0) {
                std::cout << "-----------------右转前进---------------------" << std::endl;
                dis = r + W / 2;
                angle = atan((L / 2) / dis);
            } else {
                dis = r - W / 2;
                angle = atan((L / 2) / dis);
            }
        }
    }
    if(angle > (PI / 6)) {
        angle = (PI / 6);
    } else if(angle < -(PI / 6)) {
        angle = -(PI / 6);
    }
    // // 把角速度转换成角度值 角速度 * 时间 / PI * 180 发送的是角度值
    // // 这里到底是 += 还是 = ？？？？？？？？？？？？？？
    // cv_msg.angular += (cv_msg.vel_angular_z * (1 / cv_msg.pub_frequency) / PI * 180 );

    if(cv_msg.vel_linear_x > 0.5) cv_msg.vel_linear_x = 0.5;
    else if(cv_msg.vel_linear_x < -0.5) cv_msg.vel_linear_x = -0.5;

    // if(cv_msg.angular > 30) cv_msg.angular = 30;
    // else if(cv_msg.angular < -30) cv_msg.angular = -30;
    if(flag == 0) {
        socket_param.send_buf[0] = cv_msg.vel_linear_x;
        // socket_param.send_buf[1] = cv_msg.angular;
        socket_param.send_buf[1] = angle;
        socket_param.send_buf[2] = flag;
    } else {
        socket_param.send_buf[0] = v;
        // socket_param.send_buf[1] = cv_msg.angular;
        socket_param.send_buf[1] = 0;
        socket_param.send_buf[2] = flag;
    }
    

    std::string vParam = DataToJson(socket_param.send_buf[0], socket_param.send_buf[1], socket_param.send_buf[2]);
    char arr[256];

    // std::string str1 = std::to_string(socket_param.send_buf[0]);
    // std::string str2 = std::to_string(socket_param.send_buf[1]);
    // std::string str = str1 + " " + str2; 
    // //std::cout << str << std::endl;
    // char arr[str.length() + 1];
    strcpy(arr, vParam.c_str());

    int send_num;
    // 这里的长度到底是字节数还是数组长度,先按字节数写
    send_num = sendto(socket_param.sock_fd, arr, strlen(arr), 0, (struct sockaddr *)&socket_param.addr_serv, socket_param.len);
    if(send_num < 0) {
        perror("message send error!!!!!!");
        exit(1);
    }
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

    cmd_vel_msg cv_msg;
    boost::function<void(const boost::shared_ptr<geometry_msgs::Twist const>&)> callback = boost::bind(cmd_vel_msg_listener_call_back, _1, boost::ref(cv_msg), boost::ref(socket_param));
    sub = n.subscribe("/cmd_vel", 500, callback);    //订阅cmd_vel的话题，/cmd_vel
    ROS_INFO("ready!");

    // 这里用spin还是spinOnce？
    ros::spin();
    // 关闭套接字
    close(socket_param.sock_fd);
    return 0;

}
