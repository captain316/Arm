#include <ros/ros.h>

#include <controller_manager_msgs/SwitchController.h>
// #include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
 
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_broadcaster.h>
#include "ur3_move/getObjectPosition.h"
#include "ur3_move/varObjectsPosition.h"
#include "ur_msgs/SetIO.h"

/*
    本文件为识别多种物体（香蕉、苹果、橙子），并使用联合控制器进行抓取

*/


ros::Publisher pose_pub_;


// 设置关节状态
std::vector<double> setJointState(
                                    double joint_0 = M_PI_2, double joint_1 = -M_PI_2,
                                    double joint_2 = 0, double joint_3 = -M_PI_2,
                                    double joint_4 = 0, double joint_5 = 0
                                    ) 
{
    std::vector<double> jointState = {joint_0, joint_1, joint_2, 
                                        joint_3, joint_4, joint_5};
    return jointState;
    
}

// 返回目标位姿
geometry_msgs::Pose setGoal(double px = -0.000767, double py = 0.402249,double pz = 0.709150,
                            double ox = -0.707107, double oy = -0.000425, double oz = -0.000175, double ow = 0.707107
                            )
{
    geometry_msgs::Pose goalPose;
    goalPose.position.x = px;
    goalPose.position.y = py;
    goalPose.position.z = pz;
    goalPose.orientation.x = ox;
    goalPose.orientation.y = oy;
    goalPose.orientation.z = oz;
    goalPose.orientation.w = ow;
    
    return goalPose;
}

moveit_msgs::CollisionObject addObject(
                    moveit::planning_interface::MoveGroupInterface& move_group,
                    const std::string object_id, double dx = 1,double dy = 1,double dz=1,
                    double px = 1,double py= 1,double pz = 1,double ow = 1)
{
    sleep(1);
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    // std::cout << "collision_object.header.frame_id = " << collision_object.header.frame_id << std::endl;

    // The id of the object is used to identify it.
    collision_object.id = object_id;

    // 定义物体的类型、size
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dx;
    primitive.dimensions[1] = dy;
    primitive.dimensions[2] = dz;

    // 定义物体的位置(specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = ow;
    box_pose.position.x = px;
    box_pose.position.y = py;
    box_pose.position.z = pz;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    ROS_INFO("Add object '%s' ", collision_object.id.c_str());

    return collision_object;
}

moveit_msgs::AttachedCollisionObject attachObject(
                    moveit::planning_interface::MoveGroupInterface& move_group,
                    const std::string object_id, const std::string link_name, double dx = 1,double dy = 1,double dz=1,
                    double px = 1,double py= 1,double pz = 1,double ow = 1)
{
    sleep(1);
    moveit_msgs::AttachedCollisionObject attach_object;
    
    attach_object.link_name = "fingers_frame";
    attach_object.object.header.frame_id = move_group.getPlanningFrame();
    attach_object.object.id = object_id;
    
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = ow;
    box_pose.position.x = px;
    box_pose.position.y = py;
    box_pose.position.z = pz;

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dx;
    primitive.dimensions[1] = dy;
    primitive.dimensions[2] = dz;

    attach_object.object.primitives.push_back(primitive);
    attach_object.object.primitive_poses.push_back(box_pose);
    
    attach_object.object.operation = attach_object.object.ADD; 
    ROS_INFO("Attach object '%s' ",attach_object.object.id.c_str());
    return attach_object;
}

// 夹爪控制
void Gripper_control(std::string set_gripper, bool& gripper_state)
{
	//bool gripper_state=1;//夹爪状态，1为开，0为关，初始化应为开
	bool gripper_cmd = 0;
    bool set_state = 1;
	if(set_gripper == "open") gripper_cmd = 0;
	if(set_gripper == "close") gripper_cmd = 1;

	if(gripper_state == gripper_cmd)
	{
		ros::NodeHandle io;
		ros::ServiceClient client = io.serviceClient<ur_msgs::SetIO>("ur_hardware_interface/set_io");
		ur_msgs::SetIO srv;
		srv.request.fun = 1;
		srv.request.pin = 5;
		srv.request.state = 0.0;
		if (client.call(srv))
		{
			ROS_INFO("pin set to 1: %s", srv.response.success ? "success" : "fail");
		}
		else
		{
			ROS_ERROR("Failed to call service ur_hardware_interface/set_io");
            set_state=0;
		}

		ros::WallDuration(1).sleep();
		
		srv.request.state = 24.0;
		if (client.call(srv))
		{
			ROS_INFO("pin set to 0: %s", srv.response.success ? "success" : "fail");
		}
		else
		{
			ROS_ERROR("Failed to call service ur_hardware_interface/set_io");
            set_state=0;
		}

		gripper_state = !gripper_state;
        ros::WallDuration(3).sleep();
        if(set_state)
		    ROS_INFO("gripper set to %s success" ,set_gripper.c_str());
        else
            ROS_ERROR("gripper set to %s failed" ,set_gripper.c_str());

	}
	else ROS_ERROR("Gripper already been %s",set_gripper.c_str());

}

/*************
 * 切换控制器
**************/
bool switchController(std::string start_controller, std::string stop_controller )
{
    ros::NodeHandle contr;
    ros::ServiceClient client = contr.serviceClient<controller_manager_msgs::SwitchController>
                                                    ("controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers.push_back(start_controller);
    srv.request.stop_controllers.push_back(stop_controller);
    srv.request.strictness = 1;
    bool success = 0;

    if (client.call(srv)) {
        if (srv.response.ok) {
            ROS_INFO("switch to '%s' controller success!",start_controller.c_str());
            success =1;
        } else {
            ROS_ERROR("switch controller failed");
        }   
    } else {
        ROS_ERROR("Failed to call service: /controller_manager/switch_controller");
    }
    return success;
}

void cartesianPlanning(moveit::planning_interface::MoveGroupInterface& arm, 
                        geometry_msgs::Pose& targetPose, std::string label) {

    ROS_INFO("motion with cartesian planning! ! !");
    geometry_msgs::PoseStamped target_posestamp;
    geometry_msgs::PoseStamped endlink_pose = arm.getCurrentPose();
    geometry_msgs::Pose fake_pose;  
    fake_pose.orientation =  endlink_pose.pose.orientation;
    fake_pose.position = endlink_pose.pose.position;
    double erx = targetPose.position.x - endlink_pose.pose.position.x ;
    double ery = targetPose.position.y - endlink_pose.pose.position.y; 
    double erz = targetPose.position.z - endlink_pose.pose.position.z; 
    std::cout << "endlink_pose.x = " << endlink_pose.pose.position.x << ", " << "endlink_pose.y = " << endlink_pose.pose.position.y << ", endlink_pose.z = " << endlink_pose.pose.position.z << std::endl;
    std::cout << "erx = " << erx << ", ery = " << ery << ", erz = " << erz << std::endl;
    ros::Rate loop_rate(30);
    int timecount = 0;  

    // std::cout << "x = " << fake_pose.position.x << ", y = " << fake_pose.position.y << ", z = " << fake_pose.position.z << std::endl;

    // while (timecount < 30 && ros::ok()) {     
    //     fake_pose.position.z +=  erz / 60;  
                          
    //     target_posestamp.pose = fake_pose;
    //     target_posestamp.header.frame_id = "base_link";
    //     target_posestamp.header.stamp = ros::Time::now();
    //     pose_pub_.publish(target_posestamp);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     timecount++;
    // }
    std::cout << "x = " << fake_pose.position.x << ", y = " << fake_pose.position.y << ", z = " << fake_pose.position.z << std::endl;
    while (timecount < 30 && ros::ok()) {   
        fake_pose.position.x +=  erx / 90;   
                  
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        ros::spinOnce();
        loop_rate.sleep();
        timecount++;
    }
    while (timecount < 60 && ros::ok()) {   
        fake_pose.position.x +=  erx / 90;   
        fake_pose.position.y +=  ery / 60;   
        fake_pose.position.z +=  erz / 60;                    
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        ros::spinOnce();
        loop_rate.sleep();
        timecount++;
    }
    std::cout << "x = " << fake_pose.position.x << ", y = " << fake_pose.position.y << ", z = " << fake_pose.position.z << std::endl;

    while (timecount < 90 && ros::ok()) {   
        // fake_pose.position.x = targetPose.position.x;   
        // fake_pose.position.y = targetPose.position.y;   
        // fake_pose.position.z = targetPose.position.z; 
        fake_pose.position.x +=  erx / 90;   
        fake_pose.position.y +=  ery / 60;   
        fake_pose.position.z +=  erz / 60;                   
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        ros::spinOnce();
        loop_rate.sleep();
        timecount++;
    }
    sleep(2);
    if(label == "banana") {
        fake_pose.orientation = targetPose.orientation;
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        sleep(2);
        erz = -0.053; //第一次夹物体的高度 ？
    } else {
        erz = -0.08;
    }
    
    
    while(timecount < 100 && ros::ok()) {
        fake_pose.position.z +=  erz / 10; 
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        ros::spinOnce();
        loop_rate.sleep();
        timecount++;
    }
    // while (timecount < 90 && ros::ok()) {     
    //     fake_pose.position.z +=  erz / 90;  
                          
    //     target_posestamp.pose = fake_pose;
    //     target_posestamp.header.frame_id = "base_link";
    //     target_posestamp.header.stamp = ros::Time::now();
    //     pose_pub_.publish(target_posestamp);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     timecount++;
    // }
}

// 用于抓第二个及以后的
void cartesianPlanningNext(moveit::planning_interface::MoveGroupInterface& arm, 
                        geometry_msgs::Pose& targetPose, std::string label) {

    ROS_INFO("motion with cartesian planning next! ! !");
    geometry_msgs::PoseStamped target_posestamp;
    geometry_msgs::PoseStamped endlink_pose = arm.getCurrentPose();
    geometry_msgs::Pose fake_pose;  
    fake_pose.orientation =  endlink_pose.pose.orientation;
    fake_pose.position = endlink_pose.pose.position;
    double erx = targetPose.position.x - endlink_pose.pose.position.x ;
    double ery = targetPose.position.y - endlink_pose.pose.position.y; 
    double erz = targetPose.position.z - endlink_pose.pose.position.z; 
    std::cout << "endlink_pose.x = " << endlink_pose.pose.position.x << ", " << "endlink_pose.y = " << endlink_pose.pose.position.y << ", endlink_pose.z = " << endlink_pose.pose.position.z << std::endl;
    std::cout << "erx = " << erx << ", ery = " << ery << ", erz = " << erz << std::endl;
    ros::Rate loop_rate(30);
    int timecount = 0;  

    // std::cout << "x = " << fake_pose.position.x << ", y = " << fake_pose.position.y << ", z = " << fake_pose.position.z << std::endl;
    erz = 0.1;
    while(timecount < 30 && ros::ok()) {
        fake_pose.position.z +=  erz / 30; 
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        ros::spinOnce();
        loop_rate.sleep();
        timecount++;
    }
    while (timecount < 120 && ros::ok()) {   
        fake_pose.position.x +=  erx / 90; 
        fake_pose.position.y +=  ery / 90;   
                        
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        ros::spinOnce();
        loop_rate.sleep();
        timecount++;
    }

    if(label == "banana") {
        fake_pose.orientation = targetPose.orientation;
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        sleep(2);
        erz = -0.055;
    } else {
        erz = -0.045;
    }
    erz -= 0.1;  //?
    
    while(timecount < 180 && ros::ok()) {
        fake_pose.position.z +=  erz / 60; 
        target_posestamp.pose = fake_pose;
        target_posestamp.header.frame_id = "base_link";
        target_posestamp.header.stamp = ros::Time::now();
        pose_pub_.publish(target_posestamp);
        ros::spinOnce();
        loop_rate.sleep();
        timecount++;
    }
    // while (timecount < 90 && ros::ok()) {     
    //     fake_pose.position.z +=  erz / 90;  
                          
    //     target_posestamp.pose = fake_pose;
    //     target_posestamp.header.frame_id = "base_link";
    //     target_posestamp.header.stamp = ros::Time::now();
    //     pose_pub_.publish(target_posestamp);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    //     timecount++;
    // }
}


int main(int argc, char **argv)  //主函数
{
    //ros初始化节点，节点名为moveit_fk_demo
    ros::init(argc, argv, "realUR3");
    ros::NodeHandle n;
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线程
    spinner.start();
    

    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("ur_cartesian_motion_controller/target_frame",1000);

    //初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface arm("ur_arm");

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();
    std::cout << end_effector_link << std::endl;
    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
 
    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置机械臂运动的允许误差
    arm.setGoalJointTolerance(0.001);
    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.1);
    

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject plane = 
                addObject(arm, "workstation", 2, 2, 0.001, 0, 0, -0.015);
    moveit_msgs::CollisionObject pillar= 
                addObject(arm, "pillar", 0.05, 1, 1, 0.23, 0.29, 0.5);
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(plane);
    collision_objects.push_back(pillar);

    planning_scene_interface.addCollisionObjects(collision_objects);

    moveit_msgs::AttachedCollisionObject camera = attachObject(arm, "camera", "fingers_frame", 0.19, 0.05, 0.05, 0.0, 0.25, 0.77);
    std::vector<moveit_msgs::AttachedCollisionObject> attachObjects;
    attachObjects.push_back(camera);
    planning_scene_interface.applyAttachedCollisionObjects(attachObjects);

    // planning_scene_interface.applyAttachedCollisionObject();
    ROS_INFO("add object to world");
    arm.setSupportSurfaceName("workstation");//设置支撑面以放置box

    bool gripper_state = 1; //夹爪状态，1为开，0为关，初始化应为开

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move(); //规划+运动
    // Gripper_control("open", gripper_state); // 初始状态设置为开
    sleep(1);
    
    // 运动到观察位置
    arm.setNamedTarget("detetion_position");
    arm.move(); //规划+运动
    
    sleep(5);

    ros::ServiceClient client = n.serviceClient<ur3_move::varObjectsPosition>("getObjectPose");
    ur3_move::varObjectsPosition getPose;
    getPose.request.flag = true;
    if (client.call(getPose)) {
		ROS_INFO("service success.");
	} else {
		ROS_INFO("service failed!");
		return 1;
	}

    std::cout << "The num of objects is : " << getPose.response.targets_pose.size() << std::endl;
    
    bool first = true;
    for(int i = 0; i < getPose.response.targets_pose.size(); ++i) {
        
        double px = getPose.response.targets_pose[i].position.x;
        double py = getPose.response.targets_pose[i].position.y + 0.02;
        double pz = getPose.response.targets_pose[i].position.z + 0.065;
        float angle = getPose.response.angles[i] / 180 * 3.14159 * (-1);
        std::string label = getPose.response.names[i];
        // // 设置机器人终端的目标位置
        tf2::Quaternion q;
        // q.setRPY( -3.141, -0.000122, 1.571 );
        angle =  1.5708 + angle;
        
        // std::cout << "--------------- angle = " << angle << "-----------------" << std::endl;
        q.setRPY( -3.141, 0.0, angle);
        geometry_msgs::Pose target_pose;
        // target_pose = setGoal( -0.254172, 0.226390, 0.10,
        //                        q.getX(), q.getY(), q.getZ(), q.getW());
        target_pose = setGoal( px, py, pz,
                            q.getX(), q.getY(), q.getZ(), q.getW() );
        // target_pose.position.z += 0.2;
        // target_pose.position.z -= 0.04; // 移动到物体的上方2cm
        std::cout << "-----------------------------------------------------" << std::endl;
        std::cout << "label : " << label << std::endl;
        std::cout << "px = " << px << ", py = " << py << ", pz = " << pz << std::endl;
        std::cout << "ox = " << q.getX() << ", oy = " << q.getY() << ", oz = " << q.getZ() << ", ow = " << q.getW() << std::endl;

        switchController("ur_cartesian_motion_controller", "scaled_pos_joint_traj_controller");
        if(first) {
            cartesianPlanning(arm, target_pose, label);
            first = false;
        } else {
            cartesianPlanningNext(arm, target_pose, label);
        }
        
        sleep(1);
        switchController("scaled_pos_joint_traj_controller", "ur_cartesian_motion_controller");
        // 夹爪后面打开
        Gripper_control("close", gripper_state);
        sleep(2);

        // 控制机械臂先回到丢香蕉位置
        arm.setNamedTarget("str_down");
        arm.move(); //规划+运动
        // Gripper_control("open", gripper_state); // 初始状态设置为开
        sleep(1);

        Gripper_control("open", gripper_state);
        sleep(2);

    }

   
    // 控制机械臂再回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);
    //关闭并退出
    ros::shutdown(); 

    
 
    return 0;
}
