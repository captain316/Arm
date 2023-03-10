#include <odom_pub/motor_msg.h>
#include <odom_pub/odom_pub.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish");
    ros::NodeHandle n;
    // ros::Publisher chatter_pub = n.advertise<odom_pub::motor_msg>("encoder_msg", 500);
    ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("odom", 500);
    ros::Rate loop_rate(100);
    int count = 0;
    odom_pub::motor_msg encoder_msg;
    encoder_msg.driver_motor_01 = 0.;
    encoder_msg.driver_motor_02 = 0.;
    encoder_msg.driver_motor_03 = 0.;
    encoder_msg.driver_motor_04 = 0.;
    encoder_msg.steer_motor_01 = 0.;
    encoder_msg.steer_motor_02 = 0.;
    encoder_msg.steer_motor_03 = -0.;
    encoder_msg.steer_motor_04 = -0.;

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(0.01);
    // 先发布tf变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    tf::TransformBroadcaster odom_broadcaster;
    // 发布tf变换
   
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0.;
    odom.twist.twist.linear.y = 0.;
    odom.twist.twist.angular.z = 0.;

    while(ros::ok())
    {
        odom_broadcaster.sendTransform(odom_trans);
        chatter_pub.publish(odom);
        // chatter_pub.publish(encoder_msg);
        ros::spinOnce();
        loop_rate.sleep();
        std::cout << "pub" << std::endl;
        // encoder_msg.driver_motor_01 += 0.001;
        // encoder_msg.driver_motor_02 += 0.001;
        // encoder_msg.driver_motor_03 += 0.001;
        // encoder_msg.driver_motor_04 += 0.001;
    }
    return 0;
}
