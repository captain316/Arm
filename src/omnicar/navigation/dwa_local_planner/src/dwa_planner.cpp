/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*********************************************************************/
#include <dwa_local_planner/dwa_planner.h>
#include <base_local_planner/goal_functions.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// DWAPlanner并不具体实现局部路径规划算法，它使用包base_local_planner中的SimpleTrajectoryGenerator来生成预测轨迹
// 并为预测轨迹定义了一系列的的评价标准，实际的规划规则是由SimpleScoredSamplingPlanner进行的

namespace dwa_local_planner {
  void DWAPlanner::reconfigure(DWAPlannerConfig &config)
  {

    boost::mutex::scoped_lock l(configuration_mutex_);

    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

    double resolution = planner_util_->getCostmap()->getResolution();
    path_distance_bias_ = resolution * config.path_distance_bias;
    // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
    path_costs_.setScale(path_distance_bias_);
    alignment_costs_.setScale(path_distance_bias_);

    goal_distance_bias_ = resolution * config.goal_distance_bias;
    goal_costs_.setScale(goal_distance_bias_);
    goal_front_costs_.setScale(goal_distance_bias_);

    occdist_scale_ = config.occdist_scale;
    obstacle_costs_.setScale(occdist_scale_);

    stop_time_buffer_ = config.stop_time_buffer;
    oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
    forward_point_distance_ = config.forward_point_distance;
    goal_front_costs_.setXShift(forward_point_distance_);
    alignment_costs_.setXShift(forward_point_distance_);
 
    // obstacle costs can vary due to scaling footprint feature
    obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);

    twirling_costs_.setScale(config.twirling_scale);

    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;
 
    if (vx_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      vx_samp = 1;
      config.vx_samples = vx_samp;
    }
 
    if (vy_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
      vy_samp = 1;
      config.vy_samples = vy_samp;
    }
 
    if (vth_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
      vth_samp = 1;
      config.vth_samples = vth_samp;
    }
 
    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;
 

  }


  // name是规划器的名称，planner_util是规划器辅助对象，为规划器提供了代价地图，坐标变换，全局规划等算法输入，并为一些成员变量提供了初值
  DWAPlanner::DWAPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap())
  { // 构建句柄
    ros::NodeHandle private_nh("~/" + name);
    // 设置一些必要的准则对象属性
    goal_front_costs_.setStopOnFailure( false );
    alignment_costs_.setStopOnFailure( false );

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = 0.05;
    } else {
      double controller_frequency = 0;
      private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency;
      } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);
    // 设置一些必要的准则对象属性
    oscillation_costs_.resetOscillationFlags();

    bool sum_scores;
    private_nh.param("sum_scores", sum_scores, false);
    obstacle_costs_.setSumScores(sum_scores);

    // 构建一些可视化的消息发布器和对象，主要用于在rviz工具中显示轨迹信息和潜在的代价
    private_nh.param("publish_cost_grid_pc", publish_cost_grid_pc_, false);
    map_viz_.initialize(name, planner_util->getGlobalFrame(), boost::bind(&DWAPlanner::getCellCosts, this, _1, _2, _3, _4, _5, _6));

    private_nh.param("global_frame_id", frame_id_, std::string("odom"));
    
    traj_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("trajectory_cloud", 1);
    // 用于可视化采样的路径
    private_nh.param("publish_traj_pc", publish_traj_pc_, false);

    // set up all the cost functions that will be applied in order
    // (any function returning negative values will abort scoring, so the order can improve performance)
    
    /* 一些准则：
            (1) oscillation_costs_: 尽量降低机器人在原地晃动的情况
            (2) obstacle_costs_   : 防止机器人撞在障碍物上 
            (3) path_costs_       : 使机器人尽可能的贴近全局轨迹
            (4) goal_costs_       : 更倾向于选择接近目标点的轨迹
            (5) goal_front_costs_ : 尽可能的让机器人朝向局部的nose goal
            (6) alignment_costs_  : 尽可能的让机器人保持在nose path上
            (7) twirling_costs_   : 尽量不让机器人原地打转（高速走）
    */
    // 一个vector装在所有的准则对象
    std::vector<base_local_planner::TrajectoryCostFunction*> critics;
    critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
    critics.push_back(&obstacle_costs_); // discards trajectories that move into obstacles
    critics.push_back(&goal_front_costs_); // prefers trajectories that make the nose go towards (local) nose goal
    critics.push_back(&alignment_costs_); // prefers trajectories that keep the robot nose on nose path
    critics.push_back(&path_costs_); // prefers trajectories on global path
    critics.push_back(&goal_costs_); // prefers trajectories that go towards (local) goal, based on wave propagation
    critics.push_back(&twirling_costs_); // optionally prefer trajectories that don't spin

    // trajectory generators
    // 构建轨迹生成器列表
    std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);
    // 根据准则对象列表和轨迹生成器列表构建局部规划器
    scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

    private_nh.param("cheat_factor", cheat_factor_, 1.0);
  }

  // used for visualization only, total_costs are not really total costs
  bool DWAPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {

    path_cost = path_costs_.getCellCosts(cx, cy);
    goal_cost = goal_costs_.getCellCosts(cx, cy);
    occ_cost = planner_util_->getCostmap()->getCost(cx, cy);
    if (path_cost == path_costs_.obstacleCosts() ||
        path_cost == path_costs_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }

    total_cost =
        path_distance_bias_ * path_cost +
        goal_distance_bias_ * goal_cost +
        occdist_scale_ * occ_cost;
    return true;
  }

  bool DWAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    oscillation_costs_.resetOscillationFlags();
    return planner_util_->setPlan(orig_global_plan);
  }

  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool DWAPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }

  // 在生成速度控制指令，发布局部轨迹之前，先调用了DWAPlanner的成员函数updatePlanAndLocalCosts更新轨迹和局部代价
  // global_pose是当前机器人子在全局坐标系下的位姿，new_plan是在DWAPlannerROS中截断后的剩余的全局路径
  // footprint_spec是由代价地图提供的机器人脚印（就是机器人这个框）
  void DWAPlanner::updatePlanAndLocalCosts(
      const geometry_msgs::PoseStamped& global_pose,
      const std::vector<geometry_msgs::PoseStamped>& new_plan,
      const std::vector<geometry_msgs::Point>& footprint_spec) {
    global_plan_.resize(new_plan.size());
    // 先复制全局轨迹点
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }
    // 再更新障碍物、路径和目标相关的准则对象
    obstacle_costs_.setFootprint(footprint_spec);

    // costs for going away from path
    path_costs_.setTargetPoses(global_plan_);

    // costs for not going towards the local goal as much as possible
    goal_costs_.setTargetPoses(global_plan_);

    // alignment costs
    // 这个是局部的目标点
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    // 计算当前机器人到终点的距离，
    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    double sq_dist =
        (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
        (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
    // 当前机器人到目标点的方向角偏差
    double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
    // 沿着朝向终点的方向延长forward_point_distance_，forward_point_distance_是一个配置参数
    front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
      forward_point_distance_ * cos(angle_to_goal);
    front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
      sin(angle_to_goal);
    // 更新准则对象goal_front_costs_
    goal_front_costs_.setTargetPoses(front_global_plan);
    
    // keeping the nose on the path
    // 最后更新alignment_costs
    if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
      alignment_costs_.setScale(path_distance_bias_);
      // costs for robot being aligned with path (nose on path, not ju
      alignment_costs_.setTargetPoses(global_plan_);
    } else {
      // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
      alignment_costs_.setScale(0.0);
    }
  }


  /*
   * given the current state of the robot, find a good trajectory
   */
  // 获取一条最优的局部路径，并输出速度控制指令
  // global_pose是机器人当前的全局位姿，global_vel是机器人当前的速度，在函数退出后，drive_velocities将记录速度控制指令
  // 返回值是一个在各个准则对象综合评价下的最优路径
  base_local_planner::Trajectory DWAPlanner::findBestPath(
      const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& global_vel,
      geometry_msgs::PoseStamped& drive_velocities) {

    //make sure that our configuration doesn't change mid-run
    // 确保在规划过程在不能进行动态改参数 
    boost::mutex::scoped_lock l(configuration_mutex_);  

    // pos记录机器人当前位姿，vel记录机器人当前的速度，goal记录目标点 
    Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
    Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    // 用limits记录机器人的各种运动学限制
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // prepare cost functions and generators for this run
    // 根据这些临时变量初始化轨迹生成器generator_，参数vsamples是一个向量包括三个数，就是在x、y和旋转上的指令采样数量
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);

    // 根据实际规划器scored_sampling_planner_对控制指令进行采样，并从中找到最优的局部路径
    // result_traj_是DWAPlanner的一个成员变量，它将在如下的操作之后记录最优的局部路径，
    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    // all_explored记录了所有的样本路径
    std::vector<base_local_planner::Trajectory> all_explored;
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

    // 如果配置了可视化轨迹样本和代价地图，则通过traj_cloud_pub_和map_viz_发布相关主题的消息
    if(publish_traj_pc_)
    {
        sensor_msgs::PointCloud2 traj_cloud;
        traj_cloud.header.frame_id = frame_id_;
        traj_cloud.header.stamp = ros::Time::now();

        sensor_msgs::PointCloud2Modifier cloud_mod(traj_cloud);
        cloud_mod.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32,
                                          "theta", 1, sensor_msgs::PointField::FLOAT32,
                                          "cost", 1, sensor_msgs::PointField::FLOAT32);

        unsigned int num_points = 0;
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if (t->cost_<0) // 不显示不可行的轨迹
              continue;
            num_points += t->getPointsSize(); // 将有效的轨迹的各个点的总数
        }

        cloud_mod.resize(num_points);
        sensor_msgs::PointCloud2Iterator<float> iter_x(traj_cloud, "x");
        // 填充所有有效路径的所有的点
        for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
        {
            if(t->cost_<0)
                continue;
            // Fill out the plan
            for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
                double p_x, p_y, p_th;
                t->getPoint(i, p_x, p_y, p_th);
                iter_x[0] = p_x;
                iter_x[1] = p_y;
                iter_x[2] = 0.0;
                iter_x[3] = p_th;
                iter_x[4] = t->cost_;
                ++iter_x;
            }
        }
        // 发布所有有效的路径
        traj_cloud_pub_.publish(traj_cloud);
    }

    // verbose publishing of point clouds
    if (publish_cost_grid_pc_) {
      //we'll publish the visualization of the costs to rviz before returning our best trajectory
      map_viz_.publishCostCloud(planner_util_->getCostmap());
    }

    // debrief stateful scoring functions
    // 更新碰撞准则对象
    oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_vel_trans);

    //if we don't have a legal trajectory, we'll just command zero
    // 如果result_traj_的字段cost_是一个负数，意味着没有成功规划出一条局部路径，此时机器人需要停下来
    if (result_traj_.cost_ < 0) {
      drive_velocities.pose.position.x = 0;
      drive_velocities.pose.position.y = 0;
      drive_velocities.pose.position.z = 0;
      drive_velocities.pose.orientation.w = 1;
      drive_velocities.pose.orientation.x = 0;
      drive_velocities.pose.orientation.y = 0;
      drive_velocities.pose.orientation.z = 0;
    } else {  // 成功规划出路径，根据最优样本的线速度和角速度控制量输出控制指令
      drive_velocities.pose.position.x = result_traj_.xv_;
      drive_velocities.pose.position.y = result_traj_.yv_;
      drive_velocities.pose.position.z = 0;
      tf2::Quaternion q;
      q.setRPY(0, 0, result_traj_.thetav_);
      tf2::convert(q, drive_velocities.pose.orientation);
    }

    return result_traj_;
  }
};
