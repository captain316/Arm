#include <ros/ros.h>

#include <controller_manager_msgs/SwitchController.h>
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
#include <gohome/carArm.h>
#include <boost/bind.hpp>
ros::Publisher pose_pub_;

/*
    本文件为车臂协同作业的代码，还没有具体调试

*/

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
                    moveit::planning_interface::MoveGroupInterface* move_group,
                    const std::string object_id, double dx = 1,double dy = 1,double dz=1,
                    double px = 1,double py= 1,double pz = 1,double ow = 1)
{
    sleep(1);
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
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
                    moveit::planning_interface::MoveGroupInterface* move_group,
                    const std::string object_id, const std::string link_name, double dx = 1,double dy = 1,double dz=1,
                    double px = 1,double py= 1,double pz = 1,double ow = 1)
{
    sleep(1);
    moveit_msgs::AttachedCollisionObject attach_object;
    
    attach_object.link_name = "fingers_frame";
    attach_object.object.header.frame_id = move_group->getPlanningFrame();
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
bool switchController(std::string start_controller ,std::string stop_controller )
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

void cartesianPlanning(moveit::planning_interface::MoveGroupInterface* arm, 
                        geometry_msgs::Pose& targetPose) {

    ROS_INFO("motion with cartesian planning! ! !");
    geometry_msgs::PoseStamped target_posestamp;
    geometry_msgs::PoseStamped endlink_pose = arm->getCurrentPose();
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
    fake_pose.orientation = targetPose.orientation;
    target_posestamp.pose = fake_pose;
    target_posestamp.header.frame_id = "base_link";
    target_posestamp.header.stamp = ros::Time::now();
    pose_pub_.publish(target_posestamp);
    sleep(2);
    erz = -0.05;
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
static const std::string PLANNING_GROUP = "ur_arm";
class PreWork {
    public:
        
        void initialVal(ros::NodeHandle& n);
        bool gripper_state = 1; //夹爪状态，1为开，0为关，初始化应为开
        
        //初始化需要使用move group控制的机械臂中的arm group
        moveit::planning_interface::MoveGroupInterface* arm;
        
        ros::ServiceClient client; // 获取香蕉位姿

};
void PreWork::initialVal(ros::NodeHandle& n) {

    arm = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    // 用来发布物体的位姿
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("ur_cartesian_motion_controller/target_frame",1000);
    
    //获取终端link的名称
    std::string end_effector_link = arm->getEndEffectorLink();
    std::cout << end_effector_link << std::endl;
    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm->setPoseReferenceFrame(reference_frame);
 
    //当运动规划失败后，允许重新规划
    arm->allowReplanning(true);

    //设置机械臂运动的允许误差
    arm->setGoalJointTolerance(0.001);
    //设置允许的最大速度和加速度
    arm->setMaxAccelerationScalingFactor(0.5);
    arm->setMaxVelocityScalingFactor(0.1);

    // 添加障碍物
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
    arm->setSupportSurfaceName("workstation"); //设置支撑面以放置box

    // 控制机械臂先回到初始化位置
    arm->setNamedTarget("home");
    arm->move(); //规划+运动
    sleep(1);
    // 初始化获取香蕉位姿的service
    client = n.serviceClient<ur3_move::getObjectPosition>("getObjectPose"); 
}
PreWork pre;

bool moveArm(gohome::carArm::Request& req, gohome::carArm::Response& res) {
    
    if (req.location == "goal0") {
        // 运动到观察位置
        pre.arm->setNamedTarget("detetion_position");
        pre.arm->move(); //规划+运动
        sleep(5);

        // 准备获取香蕉位姿
        ur3_move::getObjectPosition getPose;
        getPose.request.flag = true;
        if (pre.client.call(getPose)) {
            ROS_INFO("service success.");
        } else {
            ROS_INFO("service failed!");
            return 1;
        }

        double px = getPose.response.target_pose.position.x;
        double py = getPose.response.target_pose.position.y;
        double pz = getPose.response.target_pose.position.z + 0.045;
        float angle = getPose.response.angle / 180 * 3.14159 * (-1);
        // // 设置机器人终端的目标位置
        tf2::Quaternion q;
        angle =  1.5708 + angle;
        
        q.setRPY( -3.141, 0.0, angle);  
        geometry_msgs::Pose target_pose;
        
        target_pose = setGoal( px, py, pz,
                            q.getX(), q.getY(), q.getZ(), q.getW() );
        std::cout << "px = " << px << ", py = " << py << ", pz = " << pz << std::endl;
        std::cout << "ox = " << q.getX() << ", oy = " << q.getY() << ", oz = " << q.getZ() << ", ow = " << q.getW() << std::endl;

        // 控制器进行转换
        switchController("ur_cartesian_motion_controller", "scaled_pos_joint_traj_controller");
        
        // 使用笛卡尔规划
        cartesianPlanning(pre.arm, target_pose);
        sleep(1);
        
        // 移动到目标位置后切换规划器
        switchController("scaled_pos_joint_traj_controller", "ur_cartesian_motion_controller");
        
        // 夹起香蕉
        Gripper_control("close", pre.gripper_state);
        sleep(2);

        // 控制机械臂再回到初始化位置
        pre.arm->setNamedTarget("home");
        pre.arm->move();
        sleep(1);

    } else if(req.location == "goal1") { // 移动到放置香蕉位置处
        // 控制机械臂回到放置香蕉位置
        pre.arm->setNamedTarget("str_down");
        pre.arm->move(); //规划+运动
        sleep(1);
        Gripper_control("open", pre.gripper_state);
        sleep(2);
        // 然后机械臂回到初始位置
        pre.arm->setNamedTarget("home");
        pre.arm->move(); //规划+运动
        sleep(1);

    } else { // 出错
         // 控制机械臂先回到初始化位置
        ROS_INFO("The error location!!!"); 
        pre.arm->setNamedTarget("home");
        pre.arm->move(); //规划+运动
        sleep(1);
        res.result = false;
        return false;
    }
    res.result = true;
    return true;
}

int main(int argc, char **argv)  //主函数
{
    //ros初始化节点，节点名为moveit_fk_demo
    ros::init(argc, argv, "moveit_fk_demo");
    ros::NodeHandle n;
    pre.initialVal(n);
    // //多线程 //打开这个会报错
    // ros::AsyncSpinner spinner(1);
    // //开启新的线程
    // spinner.start();   
    ros::ServiceServer service = n.advertiseService("GetLocation", moveArm);
    ROS_INFO("Wait for service.....");
    
    
    ros::spin();
    
    //关闭并退出
    // ros::shutdown(); 
 
    return 0;
}
