#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <odom_pub/motor_msg.h>
#include <math.h>
#include "std_msgs/Int32MultiArray.h"
/* 统一一下 
*  01------左上轮
*  02------右上轮
*  03------左下轮
*  04------右下轮
*/
#define PI 3.14159
struct car_position
{
    double pos_x, pos_y, pos_th;    // 这里的th需要是弧度
    double linear_x, linear_y;
    double angular_z;
};

struct drive_motor_velocity {
    double driver_motor_01_v, driver_motor_02_v, driver_motor_03_v, driver_motor_04_v; 
};
class odom_msgs {
    public:
        odom_msgs() 
        {
            car_length = 0.520; // m
            car_width = 0.386;  // m
            cur_time = ros::Time::now();
            last_time = ros::Time::now();
            cur_encoder_msg.driver_motor_01 = 0.; last_encoder_msg.driver_motor_01 = 0.0;
            cur_encoder_msg.driver_motor_02 = 0.; last_encoder_msg.driver_motor_02 = 0.0;
            cur_encoder_msg.driver_motor_03 = 0.; last_encoder_msg.driver_motor_03 = 0.0;
            cur_encoder_msg.driver_motor_04 = 0.; last_encoder_msg.driver_motor_04 = 0.0;

            cur_encoder_msg.steer_motor_01 = 0.; last_encoder_msg.steer_motor_01 = 0.0;
            cur_encoder_msg.steer_motor_02 = 0.; last_encoder_msg.steer_motor_02 = 0.0;
            cur_encoder_msg.steer_motor_03 = 0.; last_encoder_msg.steer_motor_03 = 0.0;
            cur_encoder_msg.steer_motor_04 = 0.; last_encoder_msg.steer_motor_04 = 0.0;

            /*
            cur_motor_v.driver_motor_01_v = last_motor_v.driver_motor_01_v = 0.0;
            cur_motor_v.driver_motor_02_v = last_motor_v.driver_motor_02_v = 0.0;
            cur_motor_v.driver_motor_03_v = last_motor_v.driver_motor_03_v = 0.0;
            cur_motor_v.driver_motor_04_v = last_motor_v.driver_motor_04_v = 0.0;
            */

            cur_pos.pos_x = 0.0;
            cur_pos.pos_y = 0.0;
            cur_pos.pos_th = 0.0;   // 这里的th需要是弧度
            cur_pos.linear_x = 0.0;
            cur_pos.linear_y = 0.0;
            cur_pos.angular_z = 0.0;
            
           /* odom.header.stamp = cur_time;
            odom.header.frame_id = "odom";
            odom.pose.pose.position.x = 0.0;
            odom.pose.pose.position.y = 0.0;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = 0.0;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.angular.z = 0.0; */
        }
        ~odom_msgs() { }
        void encoder_msg_newst_to_cur(const boost::shared_ptr<std_msgs::Int32MultiArray const> newst_msg);
        void encoder_msg_cur_to_last();
        void encoder_msg_convert();  // 将驱动轮编码器收到数据的单位转换成米(m)，转向电机编码器收到数据的单位转换成弧度(rad)
        void compute_driver_v();
        
        float car_length, car_width;
        ros::Time cur_time, last_time;
        odom_pub::motor_msg cur_encoder_msg, last_encoder_msg;
        //nav_msgs::Odometry odom;
        drive_motor_velocity cur_motor_v, last_motor_v;
        car_position cur_pos;   // 车实时的位姿
        tf::TransformBroadcaster odom_broadcaster;
};

void odom_msgs::encoder_msg_newst_to_cur( const boost::shared_ptr<std_msgs::Int32MultiArray const> newst_msg)
{
    std::cout << "newst_msg->driver_motor_01 = " << newst_msg->data[7] << std::endl;
    cur_encoder_msg.driver_motor_01 = newst_msg->data[7];
    cur_encoder_msg.driver_motor_02 = newst_msg->data[0];
    cur_encoder_msg.driver_motor_03 = newst_msg->data[4];
    cur_encoder_msg.driver_motor_04 = newst_msg->data[3];
    cur_encoder_msg.steer_motor_01 = newst_msg->data[6];
    cur_encoder_msg.steer_motor_02 = newst_msg->data[1];
    cur_encoder_msg.steer_motor_03 = newst_msg->data[5];
    cur_encoder_msg.steer_motor_04 = newst_msg->data[2];
}

void odom_msgs::encoder_msg_cur_to_last()
{
    last_encoder_msg.driver_motor_01 = cur_encoder_msg.driver_motor_01;
    last_encoder_msg.driver_motor_02 = cur_encoder_msg.driver_motor_02;
    last_encoder_msg.driver_motor_03 = cur_encoder_msg.driver_motor_03;
    last_encoder_msg.driver_motor_04 = cur_encoder_msg.driver_motor_04;
    last_encoder_msg.steer_motor_01 = cur_encoder_msg.steer_motor_01;
    last_encoder_msg.steer_motor_02 = cur_encoder_msg.steer_motor_02;
    last_encoder_msg.steer_motor_03 = cur_encoder_msg.steer_motor_03;
    last_encoder_msg.steer_motor_04 = cur_encoder_msg.steer_motor_04;
}

void odom_msgs::encoder_msg_convert() // 弧度
{
    cur_encoder_msg.steer_motor_01 = cur_encoder_msg.steer_motor_01 / (450.0 * 131072.0) * 360.0 * PI / 180.0;
    cur_encoder_msg.steer_motor_02 = cur_encoder_msg.steer_motor_02 / (450.0 * 131072.0) * 360.0 * PI / 180.0;
    cur_encoder_msg.steer_motor_03 = cur_encoder_msg.steer_motor_03 / (450.0 * 131072.0) * 360.0 * PI / 180.0;
    cur_encoder_msg.steer_motor_04 = cur_encoder_msg.steer_motor_04 / (450.0 * 131072.0) * 360.0 * PI / 180.0;
}

void odom_msgs::compute_driver_v() // m/s
{
    cur_motor_v.driver_motor_01_v = cur_encoder_msg.driver_motor_01 / (double)(5600) * (0.27463 * PI);
    cur_motor_v.driver_motor_02_v = cur_encoder_msg.driver_motor_02 / (double)(5600) * (0.27463 * PI);
    cur_motor_v.driver_motor_03_v = cur_encoder_msg.driver_motor_03 / (double)(5600) * (0.27463 * PI);
    cur_motor_v.driver_motor_04_v = cur_encoder_msg.driver_motor_04 / (double)(5600) * (0.27463* PI);
    // cur_motor_v.driver_motor_04_v = cur_encoder_msg.driver_motor_04 / (double)(5600) * (0.28 * PI) ;
    
}


