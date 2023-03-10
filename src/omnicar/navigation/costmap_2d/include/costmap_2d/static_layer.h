/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *         David V. Lu!!
 *********************************************************************/
#ifndef COSTMAP_2D_STATIC_LAYER_H_
#define COSTMAP_2D_STATIC_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>

// 静态地图部分的内容，主要是开启地图的订阅，以及bound和cost的更新，静态地图默认只更新一次
// 地图的边界也只更新一次，对于rolling地图，需要随着机器人的运动位置不断更新附近的cost，对于非rolling地图不需要

namespace costmap_2d
{

class StaticLayer : public CostmapLayer
{
public:
  StaticLayer();
  virtual ~StaticLayer();
  virtual void onInitialize();  // 初始化
  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  // 更新地图边界，是根据传过来的静态地图的边界来更新master map的边界
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  // 更新地图的数据，这个是根据传过来的静态地图的数据来更新master map的边界
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

private:
  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map); // 回调函数
  void incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  unsigned char interpretValue(unsigned char value);  // 将接收到的地图“翻译”到本层地图

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;  /// @brief frame that map is located in
  bool subscribe_to_updates_;
  bool map_received_;
  bool has_updated_data_;
  unsigned int x_, y_, width_, height_; // x_、y_是起点，width_、heigth_是宽高
  bool track_unknown_space_;
  bool use_maximum_;
  bool first_map_only_;      ///< @brief Store the first static map and reuse it on reinitializing
  bool trinary_costmap_;
  ros::Subscriber map_sub_, map_update_sub_;

  unsigned char lethal_threshold_, unknown_cost_value_;

  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_STATIC_LAYER_H_
