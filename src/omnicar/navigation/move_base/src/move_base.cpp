/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 在Movebase主体中，各层地图的更新被启动，Action的回调函数触发全局规划线程，若成功，则将全局规划结果传入局部规划器
// 循环调用局部规划，得到速度指令，控制机器人前进，直到到达目标，其间，需要判断机器人是否达到终点（若是则退出规划）、
// 机器人是否状态异常如发生震荡行为（若是则进入恢复行为）、机器人是否超时（若是则停止规划发布零速，否则重新规划）


namespace move_base {
  // 构造函数
  MoveBase::MoveBase(tf2_ros::Buffer& tf) :
    tf_(tf),    // 坐标变换TF2的接口对象
    as_(NULL),  // Action服务器
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),  // 用于全局、局部规划器的代价地图对象
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), // 加载了baseGlobalPlanner的类库
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),  // 加载了baseLocalPlanner的类库
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior"), // 加载了recoveryBehavirour的类库
    // 这三个是记录规划结果的缓存
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    // 下面的是一些控制和反映MoveBase系统状态的布尔变量
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {
    // 新建Action服务器，指针as，绑定回调函数MoveBase::executeCb，这个函数是move_base的核心
    // as_指向action服务器，当执行as_->start()时调用MoveBase::executeCb函数
    as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    //get some parameters that will be global to the move base node
    std::string global_planner, local_planner;
    // 全局和局部规划器的名称
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // parameters of make_plan service
    private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
    private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

    //set up plan triple buffer
    // 构建了轨迹规划结果缓存
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    // 创建了一个线程planner_thread_用于轨迹规划，运行planThread函数
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    //for commanding the base
    // 订阅和发布一些话题
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    // 发布MoveBaseActionGoal消息到/move_base/goal话题上
    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
    recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

    //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    //they won't get any useful information back about its status, but this is useful for tools
    //like nav_view and rviz
    // 发布geometry_msgs::PoseStamped消息到/move_base_simple/goal话题上，回调函数goalCB会处理在/move_base_simple/goal话题上接收到的消息
    // 供nav_view和rviz等仿真工具
    ros::NodeHandle simple_nh("move_base_simple");  // move_base_simple下的句柄，订阅goal话题
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    // 通过costmap_2d模块构建了全局代价地图，并通过成员函数pause暂停运行，将在必要的功能模块初始化结束后通过成员函数start()开启
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    // 通过在一开始获取的全局规划器名称global_planner构造全局规划器，并用刚刚构建的全局代价地图完成对其的初始化操作
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    // 和上面一样，构建局部代价地图和局部规划器对象
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }

    // Start actively updating costmaps based on sensor data
    // 构建了必要的功能模块后，就开始代价地图的计算了，就是根据传感器数据动态更新全局和本地的代价地图
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //advertise a service for getting a plan
    // 开启“make_plan”和“clear_costmaps”的服务
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    //advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    // 是否使用代价地图
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    //load any user specified recovery behaviors, and if that fails load the defaults
    // 用户是否指定了恢复行为，没有的话就使用默认的
    if(!loadRecoveryBehaviors(private_nh)){
      loadDefaultRecoveryBehaviors();
    }

    //initially, we'll need to make a plan
    // 系统状态进行初始化
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    // 可能有多个恢复行为，这个是恢复行为list的索引
    recovery_index_ = 0;

    //we're all set up now so we can start the action server
    // 每当有新的Action请求时，就会调用executeCb回调函数，进行导航控制
    as_->start();

    // 动态修改系统运行参数，以下是动态配置器
    dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros::NodeHandle("~"));
    dynamic_reconfigure::Server<move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level){
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    //The first time we're called, we just want to make sure we have the
    //original configuration
    if(!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if(config.restore_defaults) {
      config = default_config_;
      //if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if(planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if(controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if(config.base_global_planner != last_config_.base_global_planner) {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      //initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if(config.base_local_planner != last_config_.base_local_planner){
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      //create a local planner
      try {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s", config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    make_plan_clear_costmap_ = config.make_plan_clear_costmap;
    make_plan_add_unreachable_goal_ = config.make_plan_add_unreachable_goal;

    last_config_ = config;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal){
    ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
    //clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }


  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp){
    if(as_->isActive()){
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    //make sure we have a costmap for our planner
    if(planner_costmap_ros_ == NULL){
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if(req.start.header.frame_id.empty())
    {
        geometry_msgs::PoseStamped global_pose;
        if(!getRobotPose(global_pose, planner_costmap_ros_)){
          ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
          return false;
        }
        start = global_pose;
    }
    else
    {
        start = req.start;
    }

    if (make_plan_clear_costmap_) {
      //update the copy of the costmap the planner uses
      clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
    }

    //first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
          req.goal.pose.position.x, req.goal.pose.position.y);

      //search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution*3.0;
      if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
      for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
        for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
          for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

            //don't search again inside the current outer layer
            if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

            //search to both sides of the desired goal
            for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

              //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

              for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
                if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if(planner_->makePlan(start, p, global_plan)){
                  if(!global_plan.empty()){

                    if (make_plan_add_unreachable_goal_) {
                      //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                      //(the reachable goal should have been added by the global planner)
                      global_plan.push_back(req.goal);
                    }

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else{
                  ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    //copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for(unsigned int i = 0; i < global_plan.size(); ++i){
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    delete dsrv_;

    if(as_ != NULL)
      delete as_;

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }
  // 全局规划函数
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear(); // 初始化空plan

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {  // 判断全局代价地图
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    //get the starting pose of the robot
    // 判断能否得到机器人位姿
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    // 根据全局规划器的makePlan函数，传入机器人当前的位姿和目标点，若规划失败，返回false，全局规划在NavFn中
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg){
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try{
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch(tf2::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
          goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent& event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  // 规划线程，在构造函数中创建了一个线程planner_thread_，这个线程是用来完成路径规划的，入口函数是planThread
  void MoveBase::planThread(){
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false; // 等待唤醒为假，表示线程已经唤醒
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    // 当全局规划失败到达上限，进入恢复行为，否则线程内一直循环全局规划
    while(n.ok()){  // move_base还活着就循环
      //check if we should run the planner (the mutex is locked)
      // 检查wait_for_wake和runPlanner_，判定是否需要进行轨迹规划，如果不需要，就通过planner_cond_来阻塞线程，等待被唤醒
      while(wait_for_wake || !runPlanner_){
        //if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
        planner_cond_.wait(lock); // 线程进入等待，在wait()时主动释放锁
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();  // 记录开始规划的时间

      //time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped temp_goal = planner_goal_; // 记录当前的目标点
      lock.unlock();                                        // 释放锁
      ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

      //run planner
      planner_plan_->clear(); // 清空规划器的路径规划
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_); // 调用makePlan使用全局规划器planner_规划轨迹，
      // 规划出来的轨迹将保存在planner_plan_所指容器中

      if(gotPlan){  // 如果成功得到了一个轨迹，就用lastest_plan_记录下最新的到的轨迹，并更新系统的状态为CONTROLLING
        // 打印规划路径上的点数
        ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
        //pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;  //表示得到了一个新的全局轨迹

        ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

        //make sure we only start the controller if we still haven't reached the goal
        // 确保只有在我们还没到达目标时才启动controller以局部规划
        if(runPlanner_)
          state_ = CONTROLLING;
        if(planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      //if we didn't get a plan and we are in the planning state (the robot isn't moving)
      // 否则，意味着没有得到一条可靠的路径，如果系统状态仍然在PLANNING状态，进行多次尝试，如果还是不行，就进入CLEARING的状态
      else if(state_==PLANNING){
        ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        //check if we've tried to make a plan for over our time limit or our maximum number of retries
        //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        //is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if(runPlanner_ &&
           (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
          //we'll move into our obstacle clearing mode
          state_ = CLEARING;
          // 全局规划标志位为假
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R; // 恢复行为触发器状态设置为全局规划失败
        }

        lock.unlock();
      }

      //take the mutex for the next iteration
      // 为了下次迭代，重新对信号量加锁，并通过一个信计时器控制线程进入等待状态来消耗规划周期中的剩余的时间
      lock.lock();

      //setup sleep interface if needed
      if(planner_frequency_ > 0){
        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0)){
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }
  // 每当接收到一个Action目标（goal）的时候，都会调用这个函数，完成了全局规划和局部规划
  // 以goal消息指针为输入参数
  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
  {
    // 检查goal是否合法（接收到的goal的旋转四元数是否有效，若无效，直接返回）
    if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }
    // 获取goal在global坐标系下的坐标点
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    // 速度置0
    publishZeroVelocity();
    //we have a goal so start the planner
    // 加锁，更新系统的运行状态
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal; // 全局规划目标点
    runPlanner_ = true;   // 全局规划标志位置位
    planner_cond_.notify_one(); // 条件变量通知正在阻塞的线程，这个是唤醒plannerThread()线程函数，因为一直在阻塞
  
    lock.unlock();
    
    // 在主题“/move_base/current_goal”上发布当前目标点
    current_goal_pub_.publish(goal);

    // 构建一个定时器r，用于按照指定的控制频率controller_frequency_进行控制，如果当前代价地图是关闭的，需要重新打开
    ros::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();  // 代价地图开启
      controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    // 记录下最近一次的控制和规划时间，重置重规划计数次数
    // 上次有效的局部规划时间设为现在
    last_valid_control_ = ros::Time::now();
    // 上次有效的全局规划时间设为现在
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;  // 全局路径规划完成

    // 接下来进入导航控制的业务循环中
    ros::NodeHandle n;
    while(n.ok())
    {
      if(c_freq_change_)  // 判断是否需要修改局部规划控制频率
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if(as_->isPreemptRequested()){  // 如果触发了抢占，需要相应作出处理
        if(as_->isNewGoalAvailable()){  // 如果是因为接收到新的目标而触发了抢占行为，将接受新目标并开始新的规划和控制
          //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();  // 接受新目标
          // 新目标点是否有效
          if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }
          // 将新目标点转到global坐标系下
          goal = goalToGlobalFrame(new_goal.target_pose);

          //we'll make sure that we reset our state for the next execution cycle
          // 接收到新目标点后的一些必要操作，和之前一样
          recovery_index_ = 0;
          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one(); // 重新进行全局规划
          lock.unlock();

          //publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          //make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        } // 如果不是因为接收到新的目标而触发了抢占行为，我们认为是取消了Action人物，将挂起退出
        else {
          //if we've been preempted explicitly we need to shut things down
          resetState();

          //notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
          as_->setPreempted();

          //we'll actually return from execute after preempting
          return;
        }
      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      // 检查goal是否被转换到了全局坐标系下
      if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
        goal = goalToGlobalFrame(goal); // 需要计算goal在全局坐标系下的坐标点

        //we want to go back to the planning state for the next execution cycle
        // 并重置规划状态
        recovery_index_ = 0;  // 恢复行为索引
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }
      // 完成各种情况检查后，才真正执行导航的控制业务executeCycle 
      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now(); // 局部规划开始时间

      //the real work on pursuing a goal is done here
      // 返回一个布尔变量用于标志是否完成了Action任务，如果成功完成了任务将退出业务逻辑
      bool done = executeCycle(goal); // 局部规划

      //if we're done, then we'll return from execute
      if(done)  // 若局部规划完成，就退出函数
        return;

      //check if execution of the goal has completed in some way
      // t_diff和start用于计算executeCycle的运行时间
      // 记录局部规划到现在的时间
      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());
      // 在业务循环的最后，通过ros::Rate对象消耗掉一个控制周期中剩余的时间，如果计时发生了偏差，就给出警告
      r.sleep();
      //make sure to sleep for the remainder of our cycle time
      if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    // 如果非正常退出循环，意味着move_base节点被杀死了，需要更新Action服务器对象as_的状态
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  // 局部规划业务逻辑主体
  // 以目标点为参数
  bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal){
    // 这个锁是保证在导航过程中参数不能被修改的
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel; // 声明速度消息

    //update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose; // 声明全局位姿
    // 获得当前的机器人位姿
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;

    //push the feedback out
    // 并发布反馈消息到Action的客户端，从服务端周期反馈，把当前位姿反馈给客户端
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    // 下面用于判定机器人是否发生了移动，只有当机器人运动了足够远，才认为机器人移动了，并用last_oscillation_reset_记录了当前时间
    // 如果在未来额的某个时间点，查看last_oscillation_reset_，发现机器人长时间没有有效移动，将会触发recover的操作，尝试修复这个异常状态
    
    // 如果移动距离超过振荡距离，没有发生振荡
    if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      // 重设振荡相关的参数据，便于下次检查
      // 把最新的振荡重置设置为当前时间
      last_oscillation_reset_ = ros::Time::now();
      // 振荡位姿设为当前姿态
      oscillation_pose_ = current_position;

      //if our last recovery was caused by oscillation, we want to reset the recovery index
      // 如果上次的恢复行为是由振荡引起，就重新设置恢复行为的索引
      if(recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    // controller_costmap_ros_是局部代价地图，通过接口isCurrent()可以判断代价地图近期是否得到了更新，
    // 如果长时间没有更新，意味着机器人失去了周围的感知能力，此时移动机器人是危险的，所有通过publishZeroVelocity()控制机器人停下来并退出业务逻辑
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    // 当我们接收到一个控制计划，需要将之提供给局部规划器来具体执行计划，首先弄清除新规划的标记变量new_global_plan_，
    // 然后交换controller_plan_和latest_plan_对象，并更新计划到局部规划器中
    
    // new_global_plan_在planThread中如果得到全局轨迹成功会被设置为真
    if(new_global_plan_){
      //make sure to set the new plan flag to false
      // 重置标志位
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      // 交换规划指针
      std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      // 这个时候latest_plan_存储的是规划好的全局规划的结果，所以controller_plan_中会存放当前最新制定好的待到达的全局规划
      controller_plan_ = latest_plan_;  
      latest_plan_ = temp_plan; 
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");
      // 更新局部规划器，就是将全局规划的结果传递给局部规划器
      if(!tc_->setPlan(*controller_plan_)){ // 如果传递失败
        //ABORT and SHUTDOWN COSTMAPS
        // 出错的话，重置系统状态，退出业务逻辑
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();  // 停止全局规划线程
        runPlanner_ = false;
        lock.unlock();
        // 停止Action服务器，打印“将全局规划传递给局部规划器控制失败”
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      // 如果找到了有效的局部规划路线，且恢复行为是“全局规划失败”，重置恢复行为索引
      if(recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    // 接下来是MoveBase的状态机，一共有三种状态：PLANNING，CONTROLLING，CLEARING。
    // 在状态PLANNING下，全局规划器根据全局代价地图规划运动轨迹，此时MoveBase将等待planner给出一条路径后切换状态
    // CONTRILLING状态，局部规划器实际控制机器人沿着轨迹运动的状态，如果机器人到达了目标点，将退出业务逻辑并返回true
    // CLEARING则是一种异常处理的状态，它调用用户定义的恢复逻辑尝试清除状态空间
    switch(state_){
      //if we are in a planning state, then we'll attempt to make a plan
      case PLANNING:
        {// 这个时候全局规划还没完成，只是唤醒一个全局规划进行规划，全局规划是全局规划器在线程planner_thread_中进行的，包括状态机都是在线程中做的，我们只需等待条件变量的同志
          boost::recursive_mutex::scoped_lock lock(planner_mutex_);
          runPlanner_ = true; // 用于控制线程planner_thread_进行规划的标志
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      // 这个时候是全局规划成功，得到全局路线，执行局部规划
      case CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //check to see if we've reached our goal
        if(tc_->isGoalReached()){ // 首先判断是否到达了终点
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState(); // 如果到达了终点就重置系统状态

          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;  // 关闭线程planner_thread_停止规划
          lock.unlock();
          // 并发布Action的Result消息，告知客户端导航目标已经达成
          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        // 如果没到达终点

        //check for an oscillation condition
        // 有时候，机器人长时间在一个地方不移动，可能出现了异常的状况，这时触发一个修复机制。
        // 就会切换到CLEARING状态，并标记修复机制的触发原因是OSCILLATION_R
        if(oscillation_timeout_ > 0.0 &&
            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
        {
          publishZeroVelocity();  // 停止移动
          state_ = CLEARING;      // 状态改变
          recovery_trigger_ = OSCILLATION_R;
        }
        // 一般情况下，都是通过局部规划器获得对机器人的速度控制指令，发布控制消息，如果局部规划器没有完成控制指令的计算
        // 将会到PLANNING状态重新规划路径，如果多次尝试仍然不能完成，就进入CLEARING状态
        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        //computeVelocityCommands函数在base_local_planner中
        if(tc_->computeVelocityCommands(cmd_vel)){  // 得到局部规划器的有效控制指令
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now(); // 重新设置有效局部控制时间
          //make sure that we send the velocity command to the base 
          vel_pub_.publish(cmd_vel);  // 发布控制命令
          if(recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;  // 重置恢复行为的索引
        }
        else {  // 若速度计算失败
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          // 计算局部规划用时限制
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          //check if we've tried to find a valid control for longer than our time limit
          if(ros::Time::now() > attempt_end){ // 长时间找不到有效的控制命令，进入CLEARING状态
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();  // 停车
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else{ // 重新进行规划，进入PLANNING状态
            //otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      // CLEARING状态是为了消除一些异常，与路径规划和导航控制没有关系
      // 如果规划失败
      // 当state_为CLEARING，进入恢复行为，将调用列表中的恢复行为，进行地图清理或者旋转
      // 并将state_设置为PLANNING，下个循环将回到全局规划，重新进行
      // 下次若再进入恢复行为，则调用列表的下一个行为，以此类推
      case CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        // 如果允许使用恢复行为，且恢复行为索引值小于恢复行为数组的大小
        if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

          move_base_msgs::RecoveryStatus msg;
          msg.pose_stamped = current_position;
          msg.current_recovery_number = recovery_index_;
          msg.total_number_of_recoveries = recovery_behaviors_.size();
          msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

          recovery_status_pub_.publish(msg);
          // 开始进行恢复行为，在executeCycle的循环中一次次迭代恢复行为
          recovery_behaviors_[recovery_index_]->runBehavior();  // 执行恢复操作

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          last_oscillation_reset_ = ros::Time::now(); // 上次振荡重置时间设为现在

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          last_valid_plan_ = ros::Time::now();
          planning_retries_ = 0;
          state_ = PLANNING;

          //update the index of the next recovery behavior that we'll try
          recovery_index_++;
        }
        else{ // 如果不允许执行恢复操作或者恢复操作都执行完了，即恢复行为无效
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");
          // 反馈失败的具体信息
          if(recovery_trigger_ == CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(recovery_trigger_ == OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;
      default:  // 目标点不对？
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        // 关闭全局规划器
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back(behavior_list[i]["name"]);
            recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  //we'll load our default recovery behaviors here
  /*
    按顺序加载了“cons_clear”、“rotate”、“ags_clear”、“rotate”
    实际上，“cons_clear”和“ags_clear”都是ClearCostmapRecovery类，负责清理机器人一定范围外的costmap上的数据，
    区别在于ags_clear保留的范围更小，清理的更多，恢复行为都是按照列表顺序调用的，当前面的无效后才会调用后面的，清理更多区域
    “rotate”是RotateRecovery类，使机器人原地旋转
    有三个场景会进入恢复行为：全局规划失败、机器人震荡、局部规划失败
  
  */
  void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try{
      //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      //first, we'll load a recovery behavior to clear the costmap
      // 加载清理代价地图的恢复行为
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("conservative_reset");
      recovery_behaviors_.push_back(cons_clear);

      //next, we'll load a recovery behavior to rotate in place
      // 加载原地自转的恢复行为
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if(clearing_rotation_allowed_){
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("rotate_recovery");
        recovery_behaviors_.push_back(rotate);
      }

      //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behavior_names_.push_back("aggressive_reset");
      recovery_behaviors_.push_back(ags_clear);

      //we'll rotate in-place one more time
      if(clearing_rotation_allowed_){
        recovery_behaviors_.push_back(rotate);
        recovery_behavior_names_.push_back("rotate_recovery");
      }
    }
    catch(pluginlib::PluginlibException& ex){
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState(){  // 重置系统状态，关闭全局规划
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("move_base","Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
