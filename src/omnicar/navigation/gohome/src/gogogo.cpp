#include <ros/ros.h>  
#include <move_base_msgs/MoveBaseAction.h>  
#include <actionlib/client/simple_action_client.h>  
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector> 
#include <string> 
#include <csignal>
#include <unistd.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;  
ros::Publisher cmd_vel_pub;
MoveBaseClient ac("move_base", true); 
class NavTest{
    public:
        int rest_time = 2;
        int n_goals = 3;
        int now_goal = 1;
        std::vector<move_base_msgs::MoveBaseGoal> goals;
        
        
        NavTest();
};

void signalHandle(int signum)
{
    ROS_INFO("Stopping the robot.........");
    ac.cancelGoal();
    ros::Duration(2).sleep();
    geometry_msgs::Twist stop_vel;
    stop_vel.linear.x = 0.;
    stop_vel.linear.y = 0.;
    stop_vel.linear.z = 0.;
    stop_vel.angular.x = 0.;
    stop_vel.angular.y = 0.;
    stop_vel.angular.z = 0.;
    for (int i = 0; i < 6; i++) {
        cmd_vel_pub.publish(stop_vel);
        ros::Duration(0.333).sleep();
    }
    ros::Duration(1).sleep();
	exit(signum);
}

NavTest::NavTest() {
    //MoveBaseClient ac("move_base", true);  
    
    goals[0].target_pose.pose.position.x = 8.177;  
    goals[0].target_pose.pose.position.y = 11.037; 
    goals[0].target_pose.pose.orientation.z = -0.999;  
    goals[0].target_pose.pose.orientation.w = 0.033; 

    goals[1].target_pose.pose.position.x = -7.066;  
    goals[1].target_pose.pose.position.y = 6.324; 
    goals[1].target_pose.pose.orientation.z = -0.703;  
    goals[1].target_pose.pose.orientation.w = 0.711;  

    goals[2].target_pose.pose.position.x = 0.0;  
    goals[2].target_pose.pose.position.y = 0.0; 
    goals[2].target_pose.pose.orientation.z = 0.0;  
    goals[2].target_pose.pose.orientation.w = 1.0;

    
    ROS_INFO("Waiting for move_base action server......");
    while (!ac.waitForServer(ros::Duration(60.0))) {  
        ROS_INFO("Waiting for the move_base action server to come up");  
    }
    ROS_INFO("Connected to move base server......");

    signal(SIGINT, signalHandle);
    while (ros::ok()) {
        if(now_goal == (n_goals + 1)) {
            now_goal = 1;
        }
        move_base_msgs::MoveBaseGoal goal = goals[now_goal - 1];
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        std::cout << "Going to: "<< std::endl;   
        std::cout << "x = " << goal.target_pose.pose.position.x << std::endl; 
        std::cout << "y = " << goal.target_pose.pose.position.y << std::endl;
        std::cout << "pose_z = " << goal.target_pose.pose.orientation.z << std::endl;
        std::cout << "pose_w = " << goal.target_pose.pose.orientation.w << std::endl;

        ac.sendGoal(goal);
        bool finished_within_time = ac.waitForResult(ros::Duration(300)); 
        if(!finished_within_time) {
            ac.cancelGoal();
            ROS_INFO("Timed out achiving goal"); 
        } else {
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("move to the %dth goal", now_goal);

            } else {
                ROS_INFO("failed move to the %dth goal", now_goal);
            }
        }
        now_goal++;
        ros::Duration(rest_time).sleep();   
    }
}



int main(int argc, char** argv) {  
    ros::init(argc, argv, "go_around");  
    ros::NodeHandle n;
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 500);
    NavTest goaround;
    ros::spin();
    return 0;  
}  
