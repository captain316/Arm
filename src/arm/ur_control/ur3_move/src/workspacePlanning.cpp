#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h> 
#include <moveit_msgs/DisplayRobotState.h> 
#include <moveit_msgs/DisplayTrajectory.h> 
#include <moveit_msgs/AttachedCollisionObject.h> 
#include <moveit_msgs/CollisionObject.h> 
// #include "ur3_move/getObjectPosition.h"
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

    ROS_INFO("Add object '%s' ",collision_object.id.c_str());

    return collision_object;
    
}




int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "moveit_fk_demo");
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启线程
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
 
    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);
 
    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
  
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject plane = 
                addObject(arm, "workstation", 2, 2, 0.001, 0, 0, -0.015);
    moveit_msgs::CollisionObject pillar = 
                addObject(arm, "pillar", 0.05, 1, 1, 0.23, 0.29, 0.5);
    // moveit_msgs::CollisionObject bar = 
    //             addObject(arm, "bar", 0.5, 0.05, 0.05, -0.045, 0.29, 0.975);
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(plane);
    collision_objects.push_back(pillar);
    // collision_objects.push_back(bar);

    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO("add object to world");
    arm.setSupportSurfaceName("workstation");//设置支撑面以放置box

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move(); //规划+运动
    sleep(1); //停1s钟
 
    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    
    //设置末端位姿
    tf2::Quaternion q;
    q.setRPY( -3.141, -0.000122, 1.571 );   
    target_pose = setGoal( -0.254838, 0.072893, 0.11000,
                           q.getX(), q.getY(), q.getZ(), q.getW() );
    // target_pose = setGoal( -0.313693, 0.11239, 0.105726,
    //                        0.707079, 0.707134, -0.000086, -0.0001793 );

                   
    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();
    // 将目标位姿写入
    arm.setPoseTarget(target_pose);
 
    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    //输出成功与否的信息
    ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");   
 
    //让机械臂按照规划的轨迹开始运动
    if(success)
      arm.execute(plan);
    sleep(1);
 
    // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("home");
    // arm.move();
    // sleep(1);
 
    ros::shutdown(); 
 
    return 0;
}
