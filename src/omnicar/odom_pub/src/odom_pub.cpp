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
#include <algorithm>
#include <chrono>
#include <iostream>

/*
* 待做的：收到编码器的数据之后，驱动轮的需要转换成m单位，转向电机的需要转换成弧度单位
*/

ros::Publisher odom_topic_pub;
ros::Subscriber sub;

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
    std::cout << "左上轮的值为" << odom_adv.cur_encoder_msg.driver_motor_01 << "m/s" << std::endl;
    std::cout << "右上轮的值为" << odom_adv.cur_encoder_msg.driver_motor_02 << "m/s" << std::endl;
    std::cout << "左下轮的值为" << odom_adv.cur_encoder_msg.driver_motor_03 << "m/s" << std::endl;
    std::cout << "右下轮的值为" << odom_adv.cur_encoder_msg.driver_motor_04 << "m/s" << std::endl;

    std::cout << "左上轮的速度为" << odom_adv.cur_motor_v.driver_motor_01_v << "m/s" << std::endl;
    std::cout << "右上轮的速度为" << odom_adv.cur_motor_v.driver_motor_02_v << "m/s" << std::endl;
    std::cout << "左下轮的速度为" << odom_adv.cur_motor_v.driver_motor_03_v << "m/s" << std::endl;
    std::cout << "右下轮的速度为" << odom_adv.cur_motor_v.driver_motor_04_v << "m/s" << std::endl;
    std::cout << "左上轮的角度为" << odom_adv.cur_encoder_msg.steer_motor_01 << "rad" << std::endl;
    std::cout << "右上轮的角度为" << odom_adv.cur_encoder_msg.steer_motor_02 << "rad" << std::endl;
    std::cout << "左下轮的角度为" << odom_adv.cur_encoder_msg.steer_motor_03 << "rad" << std::endl;
    std::cout << "右下轮的角度为" << odom_adv.cur_encoder_msg.steer_motor_04 << "rad" << std::endl;

    // 下面计算时是否要考虑到左转右转,主要是转向编码器得到的值是否有正负

    // 计算时通过左前轮的转向轮入手
    
    // 1、直走
    // 判断弧度
    if( abs(odom_adv.cur_encoder_msg.steer_motor_01) < 0.008 && 
        abs(odom_adv.cur_encoder_msg.steer_motor_02) < 0.008 && 
        abs(odom_adv.cur_encoder_msg.steer_motor_03) < 0.008 && 
        abs(odom_adv.cur_encoder_msg.steer_motor_04) < 0.008) {  // 在这儿加判断就行，判断工作在哪种模式下
        // 平均速度
        double velocity = (odom_adv.cur_motor_v.driver_motor_01_v + odom_adv.cur_motor_v.driver_motor_02_v +
                            odom_adv.cur_motor_v.driver_motor_03_v + odom_adv.cur_motor_v.driver_motor_04_v) / 4;
        std::cout << "直行：velocity = " << velocity << std::endl;
        // 平均路程
        double position = velocity * dt;    // 这段时间的路程
        std::cout << "position = " << position << std::endl;
        // 坐标是相对于map的，正负方向不会因为车的转向而改变
        odom_adv.cur_pos.pos_x += ( position * cos(odom_adv.cur_pos.pos_th) );  // 这里的th需要是弧度
        odom_adv.cur_pos.pos_y += ( position * sin(odom_adv.cur_pos.pos_th) );
        odom_adv.cur_pos.pos_th += 0.0;
        // //判断速度方向 //这里有问题
        // if(odom_adv.cur_pos.pos_th < 1.57 && odom_adv.cur_pos.pos_th > -1.57) {
        //     odom_adv.cur_pos.linear_x = velocity * cos(odom_adv.cur_pos.pos_th);
        // } else {
        //     odom_adv.cur_pos.linear_x = -velocity * cos(odom_adv.cur_pos.pos_th);
        // }
        // odom_adv.cur_pos.linear_y = velocity * sin(odom_adv.cur_pos.pos_th);
        odom_adv.cur_pos.linear_x = velocity;
        odom_adv.cur_pos.linear_y = 0.0;
        odom_adv.cur_pos.angular_z = 0.0;
        //std::cout << "published /odom with straight motion-----------------------------" << std::endl;
    }

    // 2、原地转弯模式 这里要看数据了，不一定一样
    // else if(abs(-odom_adv.cur_encoder_msg.steer_motor_01 - 
    //         (odom_adv.cur_encoder_msg.steer_motor_02 )) < 0.01 && 
    //         abs((odom_adv.cur_encoder_msg.steer_motor_03) - 
    //         (-odom_adv.cur_encoder_msg.steer_motor_04)) < 0.01 &&
    //         abs((-odom_adv.cur_encoder_msg.steer_motor_01 - 
    //         odom_adv.cur_encoder_msg.steer_motor_03)) < 0.01)
    else if(abs(odom_adv.cur_encoder_msg.steer_motor_01) < 0.95 &&  
            abs(odom_adv.cur_encoder_msg.steer_motor_01) > 0.54 &&
            abs(odom_adv.cur_encoder_msg.steer_motor_02) < 0.95 &&  
            abs(odom_adv.cur_encoder_msg.steer_motor_02) > 0.54 &&
            abs(odom_adv.cur_encoder_msg.steer_motor_03) < 0.95 &&  
            abs(odom_adv.cur_encoder_msg.steer_motor_03) > 0.54 &&
            abs(odom_adv.cur_encoder_msg.steer_motor_04) < 0.95 &&  
            abs(odom_adv.cur_encoder_msg.steer_motor_04) > 0.54 
            )
    {
        std::cout << "原地转弯模式下-------------------------" << std::endl;
        double velocity = (-odom_adv.cur_motor_v.driver_motor_01_v + (odom_adv.cur_motor_v.driver_motor_02_v) +
                            (-odom_adv.cur_motor_v.driver_motor_03_v) + (odom_adv.cur_motor_v.driver_motor_04_v)) / 4;
        
        // double velocity = -odom_adv.cur_motor_v.driver_motor_01_v;
        // if (abs(velocity) < 0.01) {
        //     velocity = 0;
        // }
        std::cout << "velocity = " << velocity << std::endl;
        double position = velocity * dt;
        double R = sqrt( pow(odom_adv.car_length, 2) + pow(odom_adv.car_width, 2) ) / 2;
        
        odom_adv.cur_pos.pos_x += 0;
        odom_adv.cur_pos.pos_y += 0;
        
        double pos_th = ( position /  R);
        
        odom_adv.cur_pos.pos_th += pos_th;
        
        odom_adv.cur_pos.linear_x = 0.0;
        odom_adv.cur_pos.linear_y = 0.0;
        odom_adv.cur_pos.angular_z = (velocity / R);
        // odom_adv.cur_pos.angular_z = 0;
        std::cout << "R = " << R << ",  position /  R = " << (position /  R) << std::endl;
        std::cout << "odom_adv.cur_pos.pos_th = " << odom_adv.cur_pos.pos_th * 57.2958 << std::endl;
        // if(odom_adv.cur_pos.pos_th > 3.14159)
        //     odom_adv.cur_pos.pos_th -= 6.28318; // (rad)
        // else if(odom_adv.cur_pos.pos_th < -3.14159)
        //     odom_adv.cur_pos.pos_th += 6.28318;
    }
    // 3、不走直线运动,在自由模式下
    else {
        float car_length_half = odom_adv.car_length / 2;
        float car_width_half = odom_adv.car_width / 2;
        // 左转情况, 这里判断左转右转须看编码器输出实际情况。
        if(odom_adv.cur_encoder_msg.steer_motor_01 > 0) {
            std::cout << "左转模式下-------------------------" << std::endl;
            // 先根据左上轮的转角求旋转半径
            float R = (car_length_half / tan(odom_adv.cur_encoder_msg.steer_motor_01)) + car_width_half;
            // 再根据左上轮速度求车整体速度
            double velocity = (R / sqrt(pow(car_length_half, 2) + pow((R - car_width_half), 2))) * odom_adv.cur_motor_v.driver_motor_01_v;
            std::cout << "左转模式下: veclocity = " << velocity << "-------------------------" << std::endl;
            // float r = car_length_half / tan(odom_adv.cur_encoder_msg.steer_motor_01) + car_width_half; // 转弯圆心到车中心的距离
            // float R = sqrt(pow(r, 2) + pow(car_length_half, 2));
            // double velocity = odom_adv.cur_motor_v.driver_motor_01_v * (r / (r - car_width_half));
            
            // 车移动的路程
            double position = velocity * dt;
            // 求角速度
            double w = velocity / R;
            // 转过的角度
            double dth = w * dt;
        
           /* 
            // 在baselink下分解XY的路程
            double dx = position * cos(dth);
            double dy = position * sin(dth);
            // 在全局坐标系下分解
            //odom_adv.cur_pos.pos_x += (dx * cos(odom_adv.cur_pos.pos_th) - dy * sin(odom_adv.cur_pos.pos_th));
            //odom_adv.cur_pos.pos_y += (dx * sin(odom_adv.cur_pos.pos_th) + dy * cos(odom_adv.cur_pos.pos_th));
            */
           // XY轴上路程分解
            odom_adv.cur_pos.pos_th += dth;
            if(odom_adv.cur_pos.pos_th > 3.14)
                odom_adv.cur_pos.pos_th -= 6.28; // (rad)
            else if(odom_adv.cur_pos.pos_th < -3.14)
                odom_adv.cur_pos.pos_th += 6.28; 

            odom_adv.cur_pos.pos_x += position * cos(odom_adv.cur_pos.pos_th);
            odom_adv.cur_pos.pos_y += position * sin(odom_adv.cur_pos.pos_th);
            
            // 速度分解(左转)
            // 车的速度是相对于车体的，而这里的角度是相对于地图的，所以需要判断一下(全向再打开)
            // if(odom_adv.cur_pos.pos_th < 1.57 && odom_adv.cur_pos.pos_th > -1.57) {
            //     odom_adv.cur_pos.linear_x = velocity * cos(odom_adv.cur_pos.pos_th);
            // }
            // else {
            //     odom_adv.cur_pos.linear_x = -velocity * cos(odom_adv.cur_pos.pos_th);
            // }
            // odom_adv.cur_pos.linear_y = velocity * sin(odom_adv.cur_pos.pos_th);

            odom_adv.cur_pos.linear_x = velocity;
            odom_adv.cur_pos.linear_y = 0;
            odom_adv.cur_pos.angular_z = w; //先不发布，等统一发布
            //std::cout << "published /odom with left motion-----------------------------" << std::endl;
        }
        else {  // 右转情况
            std::cout << "右转模式下-------------------------" << std::endl;
            // 先根据右上轮的转角求旋转半径，这里 R > 0 
            float R = -(car_length_half / tan(odom_adv.cur_encoder_msg.steer_motor_02)) + car_width_half;
            // 再根据右上轮速度求车整体速度，这里 velocity > 0
            double velocity = (R / sqrt(pow(car_length_half, 2) + pow((R - car_width_half), 2))) * odom_adv.cur_motor_v.driver_motor_02_v;
            std::cout << "右转模式下: veclocity = " << velocity << "-------------------------" << std::endl;
            // float r = -(car_length_half / tan(odom_adv.cur_encoder_msg.steer_motor_02)) + car_width_half;
            // float R = sqrt(pow(r, 2) + pow(car_length_half, 2));
            // double velocity = odom_adv.cur_motor_v.driver_motor_02_v * (r / (r - car_width_half));
            
            // 车移动的路程,这里 position > 0
            double position = velocity * dt;
            // 求角速度,这里 w < 0
            double w = -velocity / R;
            // 转过的角度,这里 dth < 0
            double dth = w * dt;
            
            /*
            // 在baselink下分解XY的路程
            double dx = position * cos(dth); // dx > 0
            double dy = position * sin(dth); // dy < 0

            // 在全局坐标系下分解，这个不确定是否正确，有可能符号是错的，
            odom_adv.cur_pos.pos_x += (dx * cos(odom_adv.cur_pos.pos_th) - dy * sin(odom_adv.cur_pos.pos_th));
            odom_adv.cur_pos.pos_y += (dx * sin(odom_adv.cur_pos.pos_th) + dy * cos(odom_adv.cur_pos.pos_th));
            odom_adv.cur_pos.pos_th += dth;
            */
            // XY轴上路程分解
            odom_adv.cur_pos.pos_th += dth;
            if(odom_adv.cur_pos.pos_th > 3.14)
                odom_adv.cur_pos.pos_th -= 6.28; // (rad)
            else if(odom_adv.cur_pos.pos_th < -3.14)
                odom_adv.cur_pos.pos_th += 6.28; 

            odom_adv.cur_pos.pos_x += position * cos(odom_adv.cur_pos.pos_th);
            odom_adv.cur_pos.pos_y += position * sin(odom_adv.cur_pos.pos_th);

            // 速度分解(右转) 可能有问题，待分析
            // 车的速度是相对于车体的，而这里的角度是相对于地图的，所以需要判断一下(全向再打开)
            // if(odom_adv.cur_pos.pos_th < 1.57 && odom_adv.cur_pos.pos_th > -1.57) {
            //     odom_adv.cur_pos.linear_x = velocity * cos(odom_adv.cur_pos.pos_th);
            // }
            // else {
            //     odom_adv.cur_pos.linear_x = -velocity * cos(odom_adv.cur_pos.pos_th);
            // }
            // odom_adv.cur_pos.linear_y = velocity * sin(odom_adv.cur_pos.pos_th);

            odom_adv.cur_pos.linear_x = velocity;
            odom_adv.cur_pos.linear_y = 0;
            odom_adv.cur_pos.angular_z = w; //先不发布，等统一发布
            //std::cout << "published /odom with right motion-----------------------------" << std::endl;
        }
        
    }
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
    ros::init(argc, argv, "publish_odom_topic");
    ros::NodeHandle n;
    
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
