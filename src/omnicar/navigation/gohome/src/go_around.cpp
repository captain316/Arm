#include <ros/ros.h>  
#include <move_base_msgs/MoveBaseAction.h>  
#include <actionlib/client/simple_action_client.h>  
#include <vector> 
#include <string> 

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;  


int main(int argc, char** argv){  
    ros::init(argc, argv, "go_around");  
    ros::NodeHandle n;    
    MoveBaseClient ac("move_base", true);  

    while(!ac.waitForServer(ros::Duration(60.0))) {  
        ROS_INFO("Waiting for the move_base action server to come up");  
    }  

    std::vector<move_base_msgs::MoveBaseGoal> goals;

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

    // stamp需要每次都改掉
    ros::Time start_time = ros::Time::now();
    int goals_num = 3;
    int now_goal = 1;
    while(ros::ok()) {
        if(now_goal == 3) {
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
            ROS_INFO("Timed out achiebing goal"); 
        } else {
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("move to the %dth goal", now_goal);
            } else {
                ROS_INFO("failed move to the %dth goal", now_goal);
            }
        }
        double running_time = (ros::Time::now() - start_time).toSec();
        ROS_INFO("Running time: %f s, and success so far: %d / %d goals", running_time, now_goal, goals_num);

        ros::Duration(2).sleep();
    }
    
    return 0;  
}  
