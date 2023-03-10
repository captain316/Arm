#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <odom_pub/motor_msg.h>
#include <odom_pub/odom_pub.h>
#include <math.h>
#include <iostream>
#include <csignal>
#include <algorithm>
#include <chrono>

using namespace std;
/*
* 待做的：收到编码器的数据之后，驱动轮的需要转换成m单位，转向电机的需要转换成弧度单位
*/

ros::Publisher odom_topic_pub;
ros::Subscriber sub;

void signalHandle(int signum)
{
	cout << "程序停止" << endl;
	exit(signum);
}

void publish_odometry_msg(const boost::shared_ptr<std_msgs::Int32MultiArray const> msg, odom_msgs& odom_adv)
{
    auto start = std::chrono::steady_clock::now();
    odom_adv.cur_time = ros::Time::now();
    double dt = (odom_adv.cur_time - odom_adv.last_time).toSec();
    
    odom_adv.last_time = odom_adv.cur_time;

    odom_adv.encoder_msg_newst_to_cur(msg); // 更新数据
    odom_adv.encoder_msg_convert();         // 转向转换成弧度值
    odom_adv.compute_driver_v();            // 计算速度
    odom_adv.encoder_msg_cur_to_last();     // 将这次的值保留
    
    std::cout << "---------------------------------------------------" << std::endl;
    std::cout << "dt = " << dt << "s" << std::endl;
    std::cout << "左上轮的值为" << odom_adv.cur_encoder_msg.driver_motor_01 << std::endl;
    std::cout << "右上轮的值为" << odom_adv.cur_encoder_msg.driver_motor_02 << std::endl;
    std::cout << "左下轮的值为" << odom_adv.cur_encoder_msg.driver_motor_03 << std::endl;
    std::cout << "右下轮的值为" << odom_adv.cur_encoder_msg.driver_motor_04 << std::endl;

    std::cout << "左上轮的速度为" << odom_adv.cur_motor_v.driver_motor_01_v << "m/s" << std::endl;
    std::cout << "右上轮的速度为" << odom_adv.cur_motor_v.driver_motor_02_v << "m/s" << std::endl;
    std::cout << "左下轮的速度为" << odom_adv.cur_motor_v.driver_motor_03_v << "m/s" << std::endl;
    std::cout << "右下轮的速度为" << odom_adv.cur_motor_v.driver_motor_04_v << "m/s" << std::endl;
    std::cout << "左上轮的角度为" << odom_adv.cur_encoder_msg.steer_motor_01 / 3.14159 * 180 << "rad" << std::endl;
    std::cout << "右上轮的角度为" << odom_adv.cur_encoder_msg.steer_motor_02 / 3.14159 * 180  << "rad" << std::endl;
    std::cout << "左下轮的角度为" << odom_adv.cur_encoder_msg.steer_motor_03 / 3.14159 * 180  << "rad" << std::endl;
    std::cout << "右下轮的角度为" << odom_adv.cur_encoder_msg.steer_motor_04 / 3.14159 * 180  << "rad" << std::endl;

    // 下面计算时是否要考虑到左转右转,主要是转向编码器得到的值是否有正负

    // 计算时通过左前轮的转向轮入手
    

    double vel_1_x = odom_adv.cur_motor_v.driver_motor_01_v * cos(odom_adv.cur_encoder_msg.steer_motor_01);
    double vel_1_y = odom_adv.cur_motor_v.driver_motor_01_v * sin(odom_adv.cur_encoder_msg.steer_motor_01);

    double vel_2_x = odom_adv.cur_motor_v.driver_motor_02_v * cos(odom_adv.cur_encoder_msg.steer_motor_02);
    double vel_2_y = odom_adv.cur_motor_v.driver_motor_02_v * sin(odom_adv.cur_encoder_msg.steer_motor_02);
    
    double vel_3_x = odom_adv.cur_motor_v.driver_motor_03_v * cos(odom_adv.cur_encoder_msg.steer_motor_03);
    double vel_3_y = odom_adv.cur_motor_v.driver_motor_03_v * sin(odom_adv.cur_encoder_msg.steer_motor_03);

    double vel_4_x = odom_adv.cur_motor_v.driver_motor_04_v * cos(odom_adv.cur_encoder_msg.steer_motor_04);
    double vel_4_y = odom_adv.cur_motor_v.driver_motor_04_v * sin(odom_adv.cur_encoder_msg.steer_motor_04);
    
   
    double vel_x = (vel_1_x + vel_2_x + vel_3_x + vel_4_x) / 4.0;
    double vel_y = (vel_1_y + vel_2_y + vel_3_y + vel_4_y) / 4.0;
    
    double omega = ( ((vel_2_x - vel_1_x) / odom_adv.car_width) + ((vel_1_y - vel_3_y) / odom_adv.car_length) ) / 2.0;
    std::cout << "vel_x = " << vel_x << ",    vel_y = " << vel_y << ",   omega = " << omega << std::endl;
    double delta_x = (vel_x * cos(odom_adv.cur_pos.pos_th) - vel_y * sin(odom_adv.cur_pos.pos_th)) * dt;
    double delta_y = (vel_x * sin(odom_adv.cur_pos.pos_th) + vel_y * cos(odom_adv.cur_pos.pos_th)) * dt;
    double delta_th = omega * dt;   

    odom_adv.cur_pos.pos_x += delta_x;
    odom_adv.cur_pos.pos_y += delta_y;
    odom_adv.cur_pos.pos_th += delta_th;

    odom_adv.cur_pos.linear_x = vel_x;
    odom_adv.cur_pos.linear_y = vel_y;
    odom_adv.cur_pos.angular_z = omega; //先不发布，等统一发布
    
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(odom_adv.cur_pos.pos_th);
    // 先发布tf变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = odom_adv.cur_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = odom_adv.cur_pos.pos_x;
    odom_trans.transform.translation.y = odom_adv.cur_pos.pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    // 发布tf变换
    odom_adv.odom_broadcaster.sendTransform(odom_trans);
    //std::cout << "publish tf_data" << std::endl;
    // 接着发布odom话题
    nav_msgs::Odometry odom;
    odom.header.stamp = odom_adv.cur_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = odom_adv.cur_pos.pos_x;
    odom.pose.pose.position.y = odom_adv.cur_pos.pos_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;


    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = odom_adv.cur_pos.linear_x;
    odom.twist.twist.linear.y = odom_adv.cur_pos.linear_y;
    odom.twist.twist.angular.z = odom_adv.cur_pos.angular_z;

    // 发布odom消息
    odom_topic_pub.publish(odom);
    auto end = std::chrono::steady_clock::now();
	// ros::Duration(0.03).sleep();	
	std::cout << "-------------time = " << std::chrono::duration<double>(end - start).count() << std::endl;
    //std::cout << "publish odom_data" << std::endl; 
}

//void pose2D_listener_call_back(const geometry_msgs::Pose2D& msg, odom_msgs& odom_pub)
void encoder_msg_listener_call_back(const boost::shared_ptr<std_msgs::Int32MultiArray const> msg, odom_msgs& odom_pub)
{
    std::cout << "recived encoder_msg ----------------------------" << std::endl;
    publish_odometry_msg(msg, odom_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_odom");
    ros::NodeHandle n;
    signal(SIGINT, signalHandle);
    odom_msgs odom_adv;         // 提醒一下，odom发布的速度要快
    odom_topic_pub = n.advertise<nav_msgs::Odometry>("odom", 500);  //发布odom的话题
    // boost::function<void(const boost::shared_ptr<odom_pub::motor_msg const>&)> callback = boost::bind(encoder_msg_listener_call_back, _1, boost::ref(odom_adv));
    boost::function<void(const boost::shared_ptr<std_msgs::Int32MultiArray const>&)> callback = boost::bind(encoder_msg_listener_call_back, _1, boost::ref(odom_adv));
    //sub = n.subscribe("/pose2D", 500, boost::bind(&pose2D_listener_call_back, std::placeholders::_1, std::ref(odom_pub)));
    sub = n.subscribe("/encoder_msg", 500, callback);    //订阅编码器的话题，/encoder_msg

    ROS_INFO("ready!");
    // 这里用spin还是spinOnce？
    ros::spin();
    return 0;

}
