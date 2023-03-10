#include <ros/ros.h>  
#include <move_base_msgs/MoveBaseAction.h>  
#include <actionlib/client/simple_action_client.h>  
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector> 
#include <string> 
#include <csignal>
#include <unistd.h>
#include <gohome/carArm.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;  


MoveBaseClient ac("move_base", true);
ros::Publisher cmd_vel_pub;

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


bool gotoLocation(move_base_msgs::MoveBaseGoal goal, int now_goal, MoveBaseClient& ac) {
    ros::Time start_time = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    std::cout << "------------- Going to the first location: --------------" << std::endl;   
    std::cout << "x = " << goal.target_pose.pose.position.x << std::endl; 
    std::cout << "y = " << goal.target_pose.pose.position.y << std::endl;
    std::cout << "pose_z = " << goal.target_pose.pose.orientation.z << std::endl;
    std::cout << "pose_w = " << goal.target_pose.pose.orientation.w << std::endl;

    ac.sendGoal(goal);
    bool finished_within_time = ac.waitForResult(ros::Duration(600)); 
    if(!finished_within_time) {
        ac.cancelGoal();
        ROS_INFO("Timed out achiebing goal"); 
    } else {
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("move to the %dth goal", now_goal + 1);
        } else {
            ROS_INFO("failed move to the %dth goal", now_goal + 1);
        }
    }
    double running_time = (ros::Time::now() - start_time).toSec();
    ROS_INFO("Running time: %f s", running_time);   // 到达物体目标点

}

int main(int argc, char** argv){  
    ros::init(argc, argv, "go_around");  
    ros::NodeHandle n;    
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 500);
    ros::ServiceClient client = n.serviceClient<gohome::carArm>("GetLocation");

    while(!ac.waitForServer(ros::Duration(60.0))) {  
        ROS_INFO("Waiting for the move_base action server to come up");  
    }  

    std::vector<move_base_msgs::MoveBaseGoal> goals(3);

    // 0位置是物体所在位置
    goals[0].target_pose.pose.position.x = 8.177;  
    goals[0].target_pose.pose.position.y = 11.037; 
    goals[0].target_pose.pose.orientation.z = -0.999;  
    goals[0].target_pose.pose.orientation.w = 0.033; 

    // 1位置是物体需要放置处
    goals[1].target_pose.pose.position.x = -7.066;  
    goals[1].target_pose.pose.position.y = 6.324; 
    goals[1].target_pose.pose.orientation.z = -0.703;  
    goals[1].target_pose.pose.orientation.w = 0.711;  

    // 2位置是机器人的原始位置
    goals[2].target_pose.pose.position.x = -7.066;  
    goals[2].target_pose.pose.position.y = 6.324; 
    goals[2].target_pose.pose.orientation.z = -0.703;  
    goals[2].target_pose.pose.orientation.w = 0.711;

    signal(SIGINT, signalHandle);

    // stamp需要每次都改掉
    ros::Time start_time = ros::Time::now();
    int now_goal = 0;
    int goal_num = goals.size();
        
    move_base_msgs::MoveBaseGoal goal = goals[now_goal];

    bool result = gotoLocation(goal, now_goal, ac);
    if(result) {
        ROS_INFO("the first goal went as planned！"); 
    } else {
        ROS_INFO("the first goal ERROR OCCUR!!!  STOP NOW!!!"); 
    }
    now_goal = ((++now_goal) % goal_num);
    ros::Duration(2).sleep(); // 到达目标物体位置


    // 向机械臂方面发送一个service，让那边进行抓取
    gohome::carArm command0;
    command0.request.location = "goal0";
    if (client.call(command0)) {
		ROS_INFO("carArm service command0 success.");
	} else {
		ROS_INFO("carArm service command0 failed!");
		return 1;
	}
    
    ros::Duration(2).sleep(); // 等抓取后等待2秒钟

    // 回复完之后我这边再进行移动到放置物体处

    goal = goals[now_goal];

    result = gotoLocation(goal, now_goal, ac);
    if(result) {
        ROS_INFO("the second goal went as planned！"); 
    } else {
        ROS_INFO("the second goal ERROR OCCUR!!!  STOP NOW!!!"); 
    }
    now_goal = ((++now_goal) % goal_num);
    ros::Duration(2).sleep(); // 到达放置物体处

    // 等这边移动到目的地，向机械臂那边发送service， 让那边把物体放下来

    gohome::carArm command1;
    command0.request.location = "goal1";
    if (client.call(command1)) {
		ROS_INFO("carArm service command1 success.");
	} else {
		ROS_INFO("carArm service command1 failed!");
		return 1;
	}
    

    goal = goals[now_goal];

    result = gotoLocation(goal, now_goal, ac);
    if(result) {
        ROS_INFO("the second goal went as planned！"); 
    } else {
        ROS_INFO("the second goal ERROR OCCUR!!!  STOP NOW!!!"); 
    }
    now_goal = ((++now_goal) % goal_num);
    ros::Duration(2).sleep(); // 到达机器人初始位置处

    
    return 0;  
}  
