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

#include <base_local_planner/simple_scored_sampling_planner.h>

#include <ros/console.h>

namespace base_local_planner {
  
  SimpleScoredSamplingPlanner::SimpleScoredSamplingPlanner(std::vector<TrajectorySampleGenerator*> gen_list, std::vector<TrajectoryCostFunction*>& critics, int max_samples) {
    max_samples_ = max_samples;
    gen_list_ = gen_list;
    critics_ = critics;
  }

  // 给轨迹打分
  double SimpleScoredSamplingPlanner::scoreTrajectory(Trajectory& traj, double best_traj_cost) {
    double traj_cost = 0;
    int gen_id = 0;
    // 遍历每个准则对象
    for(std::vector<TrajectoryCostFunction*>::iterator score_function = critics_.begin(); score_function != critics_.end(); ++score_function) {
      TrajectoryCostFunction* score_function_p = *score_function;
      if (score_function_p->getScale() == 0) {  // 如果这个准则对象的权重率为0，则分数为0跳过
        continue;
      }
      // 得到这个准则对象的代价
      double cost = score_function_p->scoreTrajectory(traj);
      if (cost < 0) { // 如果小于0，就代表这个轨迹不可用，直接退出循环
        ROS_DEBUG("Velocity %.3lf, %.3lf, %.3lf discarded by cost function  %d with cost: %f", traj.xv_, traj.yv_, traj.thetav_, gen_id, cost);
        traj_cost = cost;
        break;
      }
      // 如果代价不等于0，就乘系数
      if (cost != 0) {
        cost *= score_function_p->getScale();
      }
      traj_cost += cost;  // 累加上每个代价
      if (best_traj_cost > 0) { 
        // since we keep adding positives, once we are worse than the best, we will stay worse
        // 如果现在的代价已经比最好的要高了，就直接退出不计算其他的了
        if (traj_cost > best_traj_cost) {
          break;
        }
      }
      gen_id ++;  // 这个是第几个准则对象
    }


    return traj_cost; // 返回值是这个轨迹的代价值
  }

  // 找到最优轨迹
  // traj是输出参数代表最优的轨迹，all_explored是输出参数，代表所有的轨迹
  bool SimpleScoredSamplingPlanner::findBestTrajectory(Trajectory& traj, std::vector<Trajectory>* all_explored) {
    Trajectory loop_traj;
    Trajectory best_traj;
    double loop_traj_cost, best_traj_cost = -1;
    bool gen_success;
    int count, count_valid;
    // 遍历所有的准则对象，只要有一个没准备好就报错
    for (std::vector<TrajectoryCostFunction*>::iterator loop_critic = critics_.begin(); loop_critic != critics_.end(); ++loop_critic) {
      TrajectoryCostFunction* loop_critic_p = *loop_critic;
      if (loop_critic_p->prepare() == false) {
        ROS_WARN("A scoring function failed to prepare");
        return false;
      }
    }
    // 遍历所有的轨迹生成器，按道理应该就1个
    for (std::vector<TrajectorySampleGenerator*>::iterator loop_gen = gen_list_.begin(); loop_gen != gen_list_.end(); ++loop_gen) {
      // 重置count和count_valid
      count = 0;
      count_valid = 0;
      TrajectorySampleGenerator* gen_ = *loop_gen;
      // 判断是否还有可用的样本
      while (gen_->hasMoreTrajectories()) {
        // nextTrajectory生成轨迹，loop_traj为生成的这条轨迹（轨迹是根据一个速度和仿真时间生成的）
        gen_success = gen_->nextTrajectory(loop_traj);
        if (gen_success == false) {
          // TODO use this for debugging
          continue;
        }
        // 然后计算轨迹代价，如果代价小于最优轨迹的代价，就替换掉
        loop_traj_cost = scoreTrajectory(loop_traj, best_traj_cost);
        if (all_explored != NULL) {
          loop_traj.cost_ = loop_traj_cost;
          all_explored->push_back(loop_traj);
        }
        // 判断轨迹是否有效，代价值小于0说明轨迹无效
        if (loop_traj_cost >= 0) {
          count_valid++;  // 累加有效轨迹数
          // 如果代价小于最优轨迹的代价，就替换掉
          if (best_traj_cost < 0 || loop_traj_cost < best_traj_cost) {
            best_traj_cost = loop_traj_cost;
            best_traj = loop_traj;
          }
        }
        count++;  // 轨迹数累计
        if (max_samples_ > 0 && count >= max_samples_) {
          break;
        }        
      }
      // 遍历完一个采样器，得到了一条最优路径，就记录下来这条路径
      if (best_traj_cost >= 0) {
        traj.xv_ = best_traj.xv_;
        traj.yv_ = best_traj.yv_;
        traj.thetav_ = best_traj.thetav_;
        traj.cost_ = best_traj_cost;
        traj.resetPoints();
        double px, py, pth;
        for (unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
          best_traj.getPoint(i, px, py, pth);
          traj.addPoint(px, py, pth);
        }
      }
      ROS_DEBUG("Evaluated %d trajectories, found %d valid", count, count_valid);
      if (best_traj_cost >= 0) {
        // do not try fallback generators
        break;
      }
    }
    return best_traj_cost >= 0;
  }

  
}// namespace
