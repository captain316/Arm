#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include <json/json.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("test", 1000);

    ros::Rate loop_rate(20); // Hz

    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0.0;
    cmd_msg.linear.y = 0.0;
    cmd_msg.linear.z = 0.0;
    cmd_msg.angular.x = 0.0;
    cmd_msg.angular.y = 0.0;
    cmd_msg.angular.z = 0.0; 

    while(ros::ok())
    {
        std::cout << "cmd_msg.linear.x = " << cmd_msg.linear.x << std::endl;
        std::cout << "cmd_msg.linear.y = " << cmd_msg.linear.y << std::endl;
        std::cout << "cmd_msg.angular.z = " << cmd_msg.angular.z << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        
        // if(cmd_msg.angular.z < 0.77) {
        //     cmd_msg.angular.z += 0.01;
        // } else {
        //     cmd_msg.angular.z = 0.78;
        // }
        chatter_pub.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
