/*******
 * describe：本程序为使用Nanodet识别笛卡尔空间中运动的香蕉后，进行运动控制，跟踪运动的香蕉，当香蕉静止时，进行抓取。
 * date：2022.11.14
 * author：PN
*******/

#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>
#include <cstdlib>
#include "ur_msgs/SetIO.h"
// #include "ur3_move/Stop.h"
#include "ur3_move/movingObjectPosition.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <nav_msgs/Path.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
// #define StaticTHR 0.0038
float StaticTHR = 0.0048;

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
            set_state = 0;
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
            set_state = 0;
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
            success = 1;
        } else {
            ROS_ERROR("switch controller failed");
        }   
    } else {
        ROS_ERROR("Failed to call service: /controller_manager/switch_controller");
    }
    return success;
}


class TrackBanana
{

private:
    ros::NodeHandle nh_;
    ros::Publisher pose_pub_;
    // ros::ServiceServer stop_srv_;
    // ros::Publisher path_pub_;
    // ros::Publisher real_pub_;
    tf::TransformListener listener;    
    uint32_t stop_flag = 1;
    geometry_msgs::PoseStamped init_pose;

public:
    TrackBanana() { }
    TrackBanana(geometry_msgs::PoseStamped endlink_pose) 
    {
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ur_cartesian_motion_controller/target_frame",1000);
        // stop_srv_ = nh_.advertiseService("stop_tracking", &TrackBanana::CommandCallback, this);
        // path_pub_ = nh_.advertise<nav_msgs::Path>("banana_pose_path",1000);
        // real_pub_ = nh_.advertise<nav_msgs::Path>("banana_real_path",1000);
        init_pose = endlink_pose;
    }

    void TrackMovingBanana()
    {
        ros::Rate loop_rate(30);

        geometry_msgs::PoseStamped target_posestamp;
        geometry_msgs::Pose target_pose;  
        geometry_msgs::Pose target_pose2; 
        geometry_msgs::Pose target_pose_b3s; 
        tf::StampedTransform transform;
        tf::StampedTransform transform2;
        tf::StampedTransform transform_b3s;
        // nav_msgs::Path banana_path;
        // nav_msgs::Path real_path;
        long int timecount = 0;   
        int stop_count = 0;
        float pose_err = 1.0;
        bool flag = false;
        ROS_INFO("tracking pose above moving banana");
        // while (ros::ok() && stop_flag)
        while (ros::ok() && stop_flag)
        {
            timecount++;
            if(timecount == 1) {
                flag = true;
            }
            /********* listen tf*********/
            try
            {
                listener.waitForTransform("world", "track_banana", ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform("world", "track_banana", ros::Time(0), transform);    
                listener.waitForTransform("world", "banana", ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform("world", "banana", ros::Time(0), transform2);                
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }

            if (timecount > 100)
            {
                try
                {
                    listener.lookupTransform("world", "track_banana", ros::Time::now() - ros::Duration(3.0), transform_b3s);
                }
                catch (tf::TransformException &ex)
                {
                    ROS_WARN("%s", ex.what());
                }
                poseTFToMsg(transform_b3s, target_pose_b3s);
                pose_err =  pow(target_pose_b3s.position.x - target_pose.position.x, 2) +
                            pow(target_pose_b3s.position.y - target_pose.position.y, 2) ;
                            // pow(target_pose_b3s.position.z - target_pose.position.z, 2);
            }

            poseTFToMsg(transform, target_pose);
            poseTFToMsg(transform2, target_pose2);
            std::cout << "-------- pose_err = " << pose_err << "--------" << std::endl; 
            if (pose_err <= StaticTHR)
            {                
                ROS_WARN("static banana! pose_err is %f", pose_err);
                if((++stop_count) == 100) {
                    stop_flag = 0;
                    stop_count = 0;
                    std::cout << "----------------------------------" << std::endl;
                }
            } else {
                stop_count = 0;
            }

            // target_posestamp.pose = target_pose;
            target_pose.position.x += 0.06;
            target_pose.position.y -= 0.03;
            if(flag) {
                ROS_INFO("go to object firstly!!!");
                geometry_msgs::PoseStamped target_posestamp;
                
                geometry_msgs::Pose fake_pose;  
                fake_pose.orientation =  init_pose.pose.orientation;
                fake_pose.position = init_pose.pose.position;
                double erx = target_pose.position.x - init_pose.pose.position.x;
                double ery = target_pose.position.y - init_pose.pose.position.y; 
                int times = 0;
                while(times < 90 && ros::ok()) {
                    fake_pose.position.x +=  erx / 90;  
                    fake_pose.position.y +=  ery / 90;
                    target_posestamp.pose = fake_pose;
                    target_posestamp.header.frame_id = "base_link";
                    target_posestamp.header.stamp = ros::Time::now();
                    pose_pub_.publish(target_posestamp);
                    ros::spinOnce();
                    loop_rate.sleep();
                    times++;
                }
                flag = false;
                ros::Duration(1.0).sleep();
                
                continue;
            }

            target_posestamp.pose.position = target_pose.position;
            target_posestamp.pose.orientation = init_pose.pose.orientation;
            target_posestamp.header.frame_id = "base_link";
            target_posestamp.header.stamp = ros::Time::now();

            // banana_path.header = target_posestamp.header;
            // banana_path.poses.push_back(target_posestamp);
            // path_pub_.publish(banana_path);

            pose_pub_.publish(target_posestamp);
            ros::spinOnce();
            loop_rate.sleep();

            // target_posestamp.pose = target_pose2;
            // target_posestamp.header.frame_id = "base_link";
            // target_posestamp.header.stamp = ros::Time::now();

            // real_path.header = target_posestamp.header;
            // real_path.poses.push_back(target_posestamp);
            // real_pub_.publish(real_path);

        }
        ROS_INFO("stop tracking moving banana");
    }

    void GotoStaticBanana(moveit::planning_interface::MoveGroupInterface& arm)
    {
        ros::Rate loop_rate(30);
        int timecount = 0;   
        geometry_msgs::PoseStamped target_posestamp;
        geometry_msgs::Pose target_pose;   
        tf::StampedTransform transform;
        
        ROS_INFO("Goto pose of static banana");

        /********* listen tf*********/
        try
        {
            listener.waitForTransform("world", "banana", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("world", "banana", ros::Time(0), transform);                               
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        poseTFToMsg(transform, target_pose);
        target_pose.position.y += 0.02;
        geometry_msgs::PoseStamped endlink_pose = arm.getCurrentPose();
        geometry_msgs::Pose fake_pose;

        fake_pose.orientation =  endlink_pose.pose.orientation;
        fake_pose.position = endlink_pose.pose.position;
        double erx = target_pose.position.x - endlink_pose.pose.position.x;
        double ery = target_pose.position.y - endlink_pose.pose.position.y; 
        
        // 这个实时调
        double erz = 0.15 - endlink_pose.pose.position.z;
        // 这个得改
        while (timecount < 90 && ros::ok())
        {   
            fake_pose.position.x += (erx / 90);
            fake_pose.position.y += (ery / 90);
            fake_pose.position.z += (erz / 90);

            target_posestamp.pose = fake_pose;
            target_posestamp.header.frame_id = "base_link";

            target_posestamp.header.stamp = ros::Time::now();
            pose_pub_.publish(target_posestamp);
            ros::spinOnce();
            loop_rate.sleep();
            timecount++;
        }

        // // 调整角度
        // fake_pose.orientation = target_pose.orientation;
        // target_posestamp.pose = fake_pose;
        // target_posestamp.header.frame_id = "base_link";
        // target_posestamp.header.stamp = ros::Time::now();
        // pose_pub_.publish(target_posestamp);
        
        sleep(2);


        ROS_INFO("stop goto banana");
    }
    
};



int main(int argc ,char** argv)
{

    //ros初始化节点，节点名为moveit_fk_demo
    ros::init(argc, argv, "realUR3");
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
    arm.setGoalJointTolerance(0.001);
    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.1);

    bool gripper_state = 1;//夹爪状态，1为开，0为关，初始化应为开

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
    
    

    /********* move robot *********/
     // 运动到观察位置
    arm.setNamedTarget("detetion_position");
    arm.move(); //规划+运动
    ros::WallDuration(4.0).sleep();
    
    ros::ServiceClient client = n.serviceClient<ur3_move::movingObjectPosition>("getObjectPose");
    ur3_move::movingObjectPosition getPose;
    getPose.request.send = true;
    if (client.call(getPose)) {
		ROS_INFO("service success.");
	} else {
		ROS_INFO("service failed!");
		return 1;
	}
    
    /********* switch to cartesian control *********/

    switchController("ur_cartesian_motion_controller", "scaled_pos_joint_traj_controller");

    
    /********* publish moving banana topic *********/
    geometry_msgs::PoseStamped endlink_pose = arm.getCurrentPose();
    TrackBanana tb(endlink_pose);
    tb.TrackMovingBanana();
    tb.GotoStaticBanana(arm);

    // if(ros::isShuttingDown())exit(-1);
    // ROS_INFO("stop cartesian control");

    // /********* switch to traj control *********/
    sleep(1);
    switchController("scaled_pos_joint_traj_controller","ur_cartesian_motion_controller");

    // Gripper_control("close", gripper_state);
    // sleep(1);

    // arm.setNamedTarget("str_down");
    // arm.move(); //规划+运动
    // sleep(1);
    // // 打开夹爪
    // Gripper_control("open", gripper_state);
    sleep(1);

    // 回到home
    arm.setNamedTarget("home");
    arm.move();
    
    ros::shutdown(); 

    return 0;

}