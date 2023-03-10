#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/transform_broadcaster.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_");
    ros::NodeHandle n;
    // ros::Publisher chatter_pub = n.advertise<odom_pub::motor_msg>("encoder_msg", 500);
  
    ros::Rate loop_rate(100);
   
   
    
    // 先发布tf变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "banana";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = -0.209757;
    odom_trans.transform.translation.y = 0.257605;
    odom_trans.transform.translation.z = 0.0359817;
    odom_trans.transform.rotation.x = -0.408077000;
    odom_trans.transform.rotation.y = -0.912947001;
    odom_trans.transform.rotation.z = 0.000270531;
    odom_trans.transform.rotation.w = 0.000120920;
    tf::TransformBroadcaster odom_broadcaster;
    // 发布tf变换
   
    sleep(12.5);
    tf::TransformBroadcaster broadcaster;
    while(ros::ok())
    {
        // odom_broadcaster.sendTransform(odom_trans);
        // chatter_pub.publish(odom);
         
        broadcaster.sendTransform(
            tf::StampedTransform(
            tf::Transform(tf::Quaternion(-0.40807700, -0.91294700, 0.00027053, 0.00012092), tf::Vector3(-0.209757, 0.257605, 0.0359817)),
            ros::Time::now(),"base_link", "banana"));
        // chatter_pub.publish(encoder_msg);
        ros::spinOnce();
        loop_rate.sleep();
        // std::cout << "pub" << std::endl;
        // encoder_msg.driver_motor_01 += 0.001;
        // encoder_msg.driver_motor_02 += 0.001;
        // encoder_msg.driver_motor_03 += 0.001;
        // encoder_msg.driver_motor_04 += 0.001;
    }
    return 0;
}