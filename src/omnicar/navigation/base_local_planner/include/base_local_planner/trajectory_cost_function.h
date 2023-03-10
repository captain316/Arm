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

#ifndef TRAJECTORYCOSTFUNCTION_H_
#define TRAJECTORYCOSTFUNCTION_H_

#include <base_local_planner/trajectory.h>

namespace base_local_planner {

/**
 * @class TrajectoryCostFunction
 * @brief Provides an interface for critics of trajectories
 * During each sampling run, a batch of many trajectories will be scored using such a cost function.
 * The prepare method is called before each batch run, and then for each
 * trajectory of the sampling set, score_trajectory may be called.
 */

// 在DWAPlanner中，定义了一些准则，虽然准则对象具有不同的数据类型，但是都继承于TrajectoryCostFunction

/* 一些准则：
            (1) oscillation_costs_: 尽量降低机器人在原地晃动的情况
            (2) obstacle_costs_   : 防止机器人撞在障碍物上 
            (3) path_costs_       : 使机器人尽可能的贴近全局轨迹
            (4) goal_costs_       : 更倾向于选择接近目标点的轨迹
            (5) goal_front_costs_ : 尽可能的让机器人朝向局部的nose goal
            (6) alignment_costs_  : 尽可能的让机器人保持在nose path上
            (7) twirling_costs_   : 尽量不让机器人原地打转（高速走）
*/
// 这些准则计算的都是代价，也就是说机器人沿着一条仿真轨迹运动所需要付出的代价。
// 代价越小意味着路径越优，但是代价都应当是正数，如果准则对象计算的某条路径的代价是个负数，则意味着该路径是不合理的
// 纯虚类
class TrajectoryCostFunction {
public:

  /**
   *
   * General updating of context values if required.
   * Subclasses may overwrite. Return false in case there is any error.
   */
  // 用于更新准则对象的上下文
  virtual bool prepare() = 0;

  /**
   * return a score for trajectory traj
   */
  // 对轨迹进行打分
  virtual double scoreTrajectory(Trajectory &traj) = 0;

  double getScale() {
    return scale_;
  }

  void setScale(double scale) {
    scale_ = scale;
  }

  virtual ~TrajectoryCostFunction() {}

protected:
  TrajectoryCostFunction(double scale = 1.0): scale_(scale) {}

private:
  double scale_;
};

}

#endif /* TRAJECTORYCOSTFUNCTION_H_ */
