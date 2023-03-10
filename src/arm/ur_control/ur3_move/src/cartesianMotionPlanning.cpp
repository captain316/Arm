#include <ros/ros.h>


// #include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_broadcaster.h>
#include "ur3_move/getObjectPosition.h"
#include "ur_msgs/SetIO.h"
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

int main(int argc, char **argv)  //主函数
{
    //ros初始化节点，节点名为moveit_fk_demo
    ros::init(argc, argv, "moveit_fk_demo");
    ros::NodeHandle n;
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线程
    spinner.start();
    
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
    arm.setGoalJointTolerance(0.01);
    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.1);
    

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject plane = 
                addObject(arm, "workstation", 2, 2, 0.001, 0, 0, -0.015);
    moveit_msgs::CollisionObject pillar= 
                addObject(arm, "pillar", 0.05, 1, 1, 0.23, 0.29, 0.5);
    // moveit_msgs::CollisionObject bar= 
    //             addObject(arm,"bar",0.5,0.05,0.05,-0.045,0.29,0.975);
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(plane);
    collision_objects.push_back(pillar);
    // collision_objects.push_back(bar);

    planning_scene_interface.addCollisionObjects(collision_objects);

    moveit_msgs::AttachedCollisionObject camera = attachObject(arm, "camera", "fingers_frame", 0.19, 0.05, 0.05, 0.0, 0.25, 0.77); // 0.763
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
    
    sleep(3);

    ros::ServiceClient client = n.serviceClient<ur3_move::getObjectPosition>("getObjectPose");
    ur3_move::getObjectPosition getPose;
    getPose.request.flag = true;
    if(client.call(getPose))
	{
		ROS_INFO("service success.");

	}
	else
	{
		ROS_INFO("service failed!");
		return 1;
	}
    double px = getPose.response.target_pose.position.x;
    double py = getPose.response.target_pose.position.y;
    double pz = getPose.response.target_pose.position.z - 0.025;
    float angle = getPose.response.angle / 180 * 3.14159 * (-1);

    // // 设置机器人终端的目标位置
    tf2::Quaternion q;
    // q.setRPY( -3.141, -0.000122, 1.571 ); 
    q.setRPY( -3.141, 0.0, 1.571 + angle );  
    geometry_msgs::Pose target_pose;
    // target_pose = setGoal( -0.254172, 0.226390, 0.10,
    //                        q.getX(), q.getY(), q.getZ(), q.getW());
    target_pose = setGoal( px, py, pz,
                           q.getX(), q.getY(), q.getZ(), q.getW() );
    std::cout << "px = " << px << ", py = " << py << ", pz = " << pz << std::endl;
    std::cout << "ox = " << q.getX() << ", oy = " << q.getY() << ", oz = " << q.getZ() << ", ow = " << q.getW() << std::endl;
    
    // 目标点添加
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;
    geometry_msgs::Pose target_pose1 = start_pose;
    // 起始点
    // waypoints.push_back(start_pose);
    
    target_pose1.position.z = 0.30;
    // waypoints.push_back(target_pose1);
    // x
    target_pose1.position.x += ((target_pose.position.x - start_pose.position.x) / 2);
    target_pose1.position.y += ((target_pose.position.y - start_pose.position.y) / 2);
    waypoints.push_back(target_pose1);

    

    target_pose1.position.x = target_pose.position.x;
    // waypoints.push_back(target_pose1);
    target_pose1.position.y = target_pose.position.y;
    target_pose1.position.z = 0.05;
    waypoints.push_back(target_pose1);
    
    // y
    // target_pose1.position.y = target_pose.position.y;
    // waypoints.push_back(target_pose1);

    // 角度
    target_pose1.orientation = target_pose.orientation;
    waypoints.push_back(target_pose1);

    // z
    
    // waypoints.push_back(target_pose1); 

    target_pose1.position.z = 0.0;
    waypoints.push_back(target_pose1);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;

    const double eef_step = 0.01; //终端步进值（分辨率）
    double fraction = 0; 
    int attempts = 0;
    bool move_enable = false;
    while (fraction < 1.0 && attempts < 100) {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, NULL);
        attempts += 1;
        if(attempts % 10 == 0) {
            ROS_INFO("cartesian plan trying after %d attempts...", attempts);
        }
        
    } 
    if (fraction == 1.0) {
        ROS_INFO("plan Cartesian path done (%.2f%% acheived)", fraction * 100.0);
        arm.execute(trajectory);
        
        move_enable = true;
    } else {
        ROS_INFO("Path planning failed with only  %f success after %d attempts", fraction, attempts);
        move_enable = 0;
    }
    if (move_enable) {
        Gripper_control("close", gripper_state);
        sleep(1);
        arm.setNamedTarget("str_down");
        arm.move();
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
