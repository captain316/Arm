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
 * Author: TKruse
 *********************************************************************/

#include <base_local_planner/simple_trajectory_generator.h>

#include <cmath>

#include <base_local_planner/velocity_iterator.h>

namespace base_local_planner {

void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    std::vector<Eigen::Vector3f> additional_samples,
    bool discretize_by_time) {
  initialise(pos, vel, goal, limits, vsamples, discretize_by_time);
  // add static samples if any
  sample_params_.insert(sample_params_.end(), additional_samples.begin(), additional_samples.end());
}

// 初始化函数
// pos，vel，goal分别是当前机器人的位姿、速度、目标点
// limit记录了机器人的各种运动限制
// vsamples记录了在x、y方向上和转角的指令样本数量
// discretize_by_time用于配置采样器是否需要在时间上进行离散化处理
void SimpleTrajectoryGenerator::initialise(
    const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel,
    const Eigen::Vector3f& goal,
    base_local_planner::LocalPlannerLimits* limits,
    const Eigen::Vector3f& vsamples,
    bool discretize_by_time) {
  /*
   * We actually generate all velocity sample vectors here, from which to generate trajectories later on
   */
  // 先用成员变量保存输入参数，并定义了一些局部变量来获取机器人的运动限制
  double max_vel_th = limits->max_vel_theta;
  double min_vel_th = -1.0 * max_vel_th;
  discretize_by_time_ = discretize_by_time;
  Eigen::Vector3f acc_lim = limits->getAccLimits();
  pos_ = pos;
  vel_ = vel;
  limits_ = limits;
  next_sample_index_ = 0;
  // 清空采样配置
  sample_params_.clear();

  double min_vel_x = limits->min_vel_x;
  double max_vel_x = limits->max_vel_x;
  double min_vel_y = limits->min_vel_y;
  double max_vel_y = limits->max_vel_y;

  // if sampling number is zero in any dimension, we don't generate samples generically
  // 根据样本数量的限制，只要有一个维度的样本数量为0就不再采样
  if (vsamples[0] * vsamples[1] * vsamples[2] > 0) {
    //compute the feasible velocity space based on the rate at which we run
    Eigen::Vector3f max_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f min_vel = Eigen::Vector3f::Zero();

    if ( ! use_dwa_) {  // 默认进这个
      // there is no point in overshooting the goal, and it also may break the
      // robot behavior, so we limit the velocities to those that do not overshoot in sim_time
      double dist = hypot(goal[0] - pos[0], goal[1] - pos[1]);
      max_vel_x = std::max(std::min(max_vel_x, dist / sim_time_), min_vel_x);
      max_vel_y = std::max(std::min(max_vel_y, dist / sim_time_), min_vel_y);

      // if we use continous acceleration, we can sample the max velocity we can reach in sim_time_
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_time_);
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_time_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_time_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_time_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_time_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_time_);
    } else {
      // with dwa do not accelerate beyond the first step, we only sample within velocities we reach in sim_period
      // 首先计算机器人的速度限制，并保存在局部变量max_vel和min_vel中
      max_vel[0] = std::min(max_vel_x, vel[0] + acc_lim[0] * sim_period_);
      max_vel[1] = std::min(max_vel_y, vel[1] + acc_lim[1] * sim_period_);
      max_vel[2] = std::min(max_vel_th, vel[2] + acc_lim[2] * sim_period_);

      min_vel[0] = std::max(min_vel_x, vel[0] - acc_lim[0] * sim_period_);
      min_vel[1] = std::max(min_vel_y, vel[1] - acc_lim[1] * sim_period_);
      min_vel[2] = std::max(min_vel_th, vel[2] - acc_lim[2] * sim_period_);
    }

    // 根据指定的速度样本参数，生成采样速度指令并将之保存在sample_params中
    // VelocityIterator，本质上是对速度指令空间均分为vsamples-1个空间，并依次取样本空间的最小值、最大值、以及各个区间分界点作为样本点
    Eigen::Vector3f vel_samp = Eigen::Vector3f::Zero();
    VelocityIterator x_it(min_vel[0], max_vel[0], vsamples[0]); // x_it里存放x方向速度的采样
    VelocityIterator y_it(min_vel[1], max_vel[1], vsamples[1]); // y_it里存放y方向速度的采样
    VelocityIterator th_it(min_vel[2], max_vel[2], vsamples[2]);// th_it里存放角速度的采样
    for(; !x_it.isFinished(); x_it++) {
      vel_samp[0] = x_it.getVelocity();
      for(; !y_it.isFinished(); y_it++) {
        vel_samp[1] = y_it.getVelocity();
        for(; !th_it.isFinished(); th_it++) {
          vel_samp[2] = th_it.getVelocity();
          //ROS_DEBUG("Sample %f, %f, %f", vel_samp[0], vel_samp[1], vel_samp[2]);
          sample_params_.push_back(vel_samp);
        }
        th_it.reset();
      }
      y_it.reset();
    }
  }
}

void SimpleTrajectoryGenerator::setParameters(
    double sim_time,
    double sim_granularity,
    double angular_sim_granularity,
    bool use_dwa,
    double sim_period) {
  sim_time_ = sim_time;
  sim_granularity_ = sim_granularity;
  angular_sim_granularity_ = angular_sim_granularity;
  use_dwa_ = use_dwa;
  continued_acceleration_ = ! use_dwa_;
  sim_period_ = sim_period;
}

/**
 * Whether this generator can create more trajectories
 */
// 判断是否还有样本没有计算cost
bool SimpleTrajectoryGenerator::hasMoreTrajectories() {
  return next_sample_index_ < sample_params_.size();
}

/**
 * Create and return the next sample trajectory
 */
// 返回下一条速度生成的一条轨迹，轨迹点都保存在comp_traj
bool SimpleTrajectoryGenerator::nextTrajectory(Trajectory &comp_traj) {
  bool result = false;
  if (hasMoreTrajectories()) {  // 先判断是否还有可用的样本
    if (generateTrajectory( // 如果有就通过generateTrajectory，根据样本速度指令生成对应的仿真轨迹
        pos_,               // 最后增加样本计数并返回是否成功生成轨迹
        vel_,
        sample_params_[next_sample_index_],
        comp_traj)) {
      result = true;
    }
  }
  next_sample_index_++;
  return result;
}

/**
 * @param pos current position of robot
 * @param vel desired velocity for sampling
 */
// 根据采样速度生成轨迹
// pos和vel是机器人目前的位姿和速度，sample_target_vel是样本速度指令，traj是一个输出参数，记录生成的样本轨迹
bool SimpleTrajectoryGenerator::generateTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f sample_target_vel,
      base_local_planner::Trajectory& traj) {
  // 求x和y的和速度 
  double vmag = hypot(sample_target_vel[0], sample_target_vel[1]);
  // eps是一个有限小量，用于对机器人的速度限幅作出适当的膨胀
  double eps = 1e-4;
  // 清空轨迹traj
  traj.cost_   = -1.0; // placed here in case we return early
  //trajectory might be reused so we'll make sure to reset it
  traj.resetPoints();

  // make sure that the robot would at least be moving with one of
  // the required minimum velocities for translation and rotation (if set)
  // 判断速度是否在指令空间内，不是就报错退出
  if ((limits_->min_vel_trans >= 0 && vmag + eps < limits_->min_vel_trans) &&
      (limits_->min_vel_theta >= 0 && fabs(sample_target_vel[2]) + eps < limits_->min_vel_theta)) {
    return false;
  }
  // make sure we do not exceed max diagonal (x+y) translational velocity (if set)
  if (limits_->max_vel_trans >=0 && vmag - eps > limits_->max_vel_trans) {
    return false;
  }

  int num_steps;
  if (discretize_by_time_) {  // 默认false
    num_steps = ceil(sim_time_ / sim_granularity_);
  } else {
    //compute the number of steps we must take along this trajectory to be "safe"
    // 根据仿真粒度计算仿真步数，即样本点数
    double sim_time_distance = vmag * sim_time_; // the distance the robot would travel in sim_time if it did not change velocity
    double sim_time_angle = fabs(sample_target_vel[2]) * sim_time_; // the angle the robot would rotate in sim_time
    // ceil函数用来返回一个不小于给定数字的最小整数值
    num_steps =
        ceil(std::max(sim_time_distance / sim_granularity_,
            sim_time_angle    / angular_sim_granularity_));
  }
  // 如果速度过小，在sim_time_时间内不能产生超过仿真粒度的线位移或角位移就退出
  if (num_steps == 0) {
    return false;
  }

  //compute a timestep
  // 计算仿真步长
  double dt = sim_time_ / num_steps;
  traj.time_delta_ = dt;

  Eigen::Vector3f loop_vel;
  // 如果考虑加速度，通过成员函数computeNewVelocities计算，依据当前速度、指令速度、加速度限制、以及仿真步长计算新的速度量
  // 默认进这个
  if (continued_acceleration_) {  
    // assuming the velocity of the first cycle is the one we want to store in the trajectory object
    loop_vel = computeNewVelocities(sample_target_vel, vel, limits_->getAccLimits(), dt);
    traj.xv_     = loop_vel[0];
    traj.yv_     = loop_vel[1];
    traj.thetav_ = loop_vel[2];
  } else {
    // assuming sample_vel is our target velocity within acc limits for one timestep
    // 不受加速度限制
    loop_vel = sample_target_vel;
    traj.xv_     = sample_target_vel[0];
    traj.yv_     = sample_target_vel[1];
    traj.thetav_ = sample_target_vel[2];
  }

  //simulate the trajectory and check for collisions, updating costs along the way
  // 在for循环中一次计算各个仿真步的位置点，并将之添加到输出参数traj中
  for (int i = 0; i < num_steps; ++i) {

    //add the point to the trajectory so we can draw it later if we want
    // 将这段仿真时间经过的路径点都添加到traj中，用于后面的评判cost
    traj.addPoint(pos[0], pos[1], pos[2]);

    if (continued_acceleration_) {
      //calculate velocities
      // 考虑加速度的话就得重新计算速度
      loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
      //ROS_WARN_NAMED("Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_, loop_vel[0], loop_vel[1], loop_vel[2]);
    }
    // 不考虑加速度的话就代表在仿真时间内速度都是恒定的
    //update the position of the robot using the velocities passed in
    // 计算这段仿真时间内经过的点
    pos = computeNewPositions(pos, loop_vel, dt);

  } // end for simulation steps

  return true; // trajectory has at least one point
}

// 计算新的位置点
// pos是当前位置，vel是当前速度，dt是仿真步长
Eigen::Vector3f SimpleTrajectoryGenerator::computeNewPositions(const Eigen::Vector3f& pos,
    const Eigen::Vector3f& vel, double dt) {
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

/**
 * change vel using acceleration limits to converge towards sample_target-vel
 */
// 考虑到加速度的情况下计算新的速度
// new_vel是输出速度，sample_target_vel是目标速度，vel是当前速度，dt是仿真步长
Eigen::Vector3f SimpleTrajectoryGenerator::computeNewVelocities(const Eigen::Vector3f& sample_target_vel,
    const Eigen::Vector3f& vel, Eigen::Vector3f acclimits, double dt) {
  Eigen::Vector3f new_vel = Eigen::Vector3f::Zero();
  for (int i = 0; i < 3; ++i) {
    if (vel[i] < sample_target_vel[i]) {
      new_vel[i] = std::min(double(sample_target_vel[i]), vel[i] + acclimits[i] * dt);
    } else {
      new_vel[i] = std::max(double(sample_target_vel[i]), vel[i] - acclimits[i] * dt);
    }
  }
  return new_vel;
}

} /* namespace base_local_planner */
