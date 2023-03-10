#include <laser_scan_matcher/Pose2DToOdom.h>

ros::Publisher odom_topic_pub;
ros::Subscriber sub;

void pose2D_msg_listener_call_back(const boost::shared_ptr<geometry_msgs::Pose2D const> msg, pose2Dmsg& posemsg)
{
    std::cout << "recived /pose2D topic-------------------" << std::endl;
    posemsg.x = msg->x;
    posemsg.y = msg->y;
    posemsg.theta = msg->theta;
    posemsg.cur_time = ros::Time::now();
    double dt = (posemsg.cur_time - posemsg.last_time).toSec();
    double vx = (posemsg.x - posemsg.last_x) / dt;
    double vy = (posemsg.y - posemsg.last_y) / dt;
    double vth = (posemsg.theta - posemsg.last_theta);
    posemsg.last_x = msg->x;
    posemsg.last_y = msg->y;
    posemsg.last_theta = msg->theta;
    posemsg.last_time = posemsg.cur_time;
    nav_msgs::Odometry odom;
    odom.header.stamp = posemsg.cur_time;

    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(posemsg.theta);

    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = posemsg.x;
    odom.pose.pose.position.y = posemsg.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;


    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom_topic_pub.publish(odom);
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle n;
    pose2Dmsg posemsg;
    odom_topic_pub = n.advertise<nav_msgs::Odometry>("odom", 500);
    boost::function<void(const boost::shared_ptr<geometry_msgs::Pose2D const>&)> callback = boost::bind(pose2D_msg_listener_call_back, _1, boost::ref(posemsg));
    sub = n.subscribe("/pose2D", 500, callback);    //订阅cmd_vel的话题，/cmd_vel
    ROS_INFO("ready!");
    // 这里用spin还是spinOnce？
    ros::spin();
    return 0;

}