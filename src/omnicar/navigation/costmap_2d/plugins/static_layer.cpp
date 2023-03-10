/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
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

// 静态地图部分的内容，主要是开启地图的订阅，以及bound和cost的更新，静态地图默认只更新一次
// 地图的边界也只更新一次，对于rolling地图，需要随着机器人的运动位置不断更新附近的cost，对于非rolling地图不需要

#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace costmap_2d
{

StaticLayer::StaticLayer() : dsrv_(NULL) {}

StaticLayer::~StaticLayer()
{
  if (dsrv_)
    delete dsrv_;
}

// 初始化， Costmap2DROS的构造函数中会调用各Layer中的initialize函数，而initialize函数会调用onInitialize函数，静态层真正的初始化在这儿
void StaticLayer::onInitialize()  
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID();

  // 初始化以下参数
  std::string map_topic;
  nh.param("map_topic", map_topic, std::string("map"));
  nh.param("first_map_only", first_map_only_, false);
  nh.param("subscribe_to_updates", subscribe_to_updates_, false);

  nh.param("track_unknown_space", track_unknown_space_, true);
  nh.param("use_maximum", use_maximum_, false);

  int temp_lethal_threshold, temp_unknown_cost_value;
  nh.param("lethal_cost_threshold", temp_lethal_threshold, int(100));
  nh.param("unknown_cost_value", temp_unknown_cost_value, int(-1));
  nh.param("trinary_costmap", trinary_costmap_, true);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  unknown_cost_value_ = temp_unknown_cost_value;

  // Only resubscribe if topic has changed
  // 只有当话题改变后才会重新订阅
  if (map_sub_.getTopic() != ros::names::resolve(map_topic))
  {
    // we'll subscribe to the latched topic that the map server uses
    ROS_INFO("Requesting the map...");
    // 如果订阅到了map_topic，就转到回调函数incomingMap，在回调函数中会让map_received_和has_updated_data_置为true；
    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && g_nh.ok()) // 如果没有受到地图，会在这儿阻塞，直至订阅到地图话题
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

    // 判断是否接收static map的更新（默认是false），若接受，则开启topic对map_topic+"_ipdates"的更新，最后开启参数动态配置服务 
    if (subscribe_to_updates_)
    {
      ROS_INFO("Subscribing to updates");
      map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &StaticLayer::incomingUpdate, this);

    }
  }
  else
  {
    has_updated_data_ = true;
  }

  if (dsrv_)
  {
    delete dsrv_;
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &StaticLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void StaticLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  if (config.enabled != enabled_)
  {
    enabled_ = config.enabled;
    has_updated_data_ = true;
    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
  }
}

void StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling())
  {
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }
}


// "翻译"数据
// 接收到的数据（value）: unknown_cost_value（默认为-1）为未知区域
//                     lethal_cost_threshold（默认为100）以上为致命障碍物
unsigned char StaticLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_)
    return NO_INFORMATION;  // 255  tracl_unknown_space默认为true好像
  else if (!track_unknown_space_ && value == unknown_cost_value_)
    return FREE_SPACE;      // 0
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE; // 254
  else if (trinary_costmap_)  // trinary（三元的）还不知道是什么意思
    return FREE_SPACE;
  // 以上都不是，按比例返回代价值
  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

// 获取接收到的静态地图的尺寸，当地图不随机器人移动时，若收到的静态地图和主地图的尺寸/分辨率/起点不同，以接收到的地图为准，调整主地图的参数
void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

  // resize costmap if size, resolution or origin do not match
  // 若收到的地图的尺寸/分辨率/起点和master地图不同时就以收到的为准
  Costmap2D* master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() &&
      (master->getSizeInCellsX() != size_x ||
       master->getSizeInCellsY() != size_y ||
       master->getResolution() != new_map->info.resolution ||
       master->getOriginX() != new_map->info.origin.position.x ||
       master->getOriginY() != new_map->info.origin.position.y))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    ROS_INFO("Resizing costmap to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    layered_costmap_->resizeMap(size_x, size_y, new_map->info.resolution, new_map->info.origin.position.x,
                                new_map->info.origin.position.y,
                                true /* set size_locked to true, prevents reconfigureCb from overriding map size*/);
  } // 若本层的地图参数和接收到的静态地图的参数不同时，继续以接收到的地图为准，调整本层参数
  else if (size_x_ != size_x || size_y_ != size_y ||
           resolution_ != new_map->info.resolution ||
           origin_x_ != new_map->info.origin.position.x ||
           origin_y_ != new_map->info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
    resizeMap(size_x, size_y, new_map->info.resolution,
              new_map->info.origin.position.x, new_map->info.origin.position.y);
  }

  // 将接收到的静态地图数据复制到本层地图，复制过程中调用interpretValue函数进行“翻译”，
  // 并设置好地图坐标系中的原点x_、y_、以及地图尺寸和标志位
  unsigned int index = 0;

  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;  // 获取坐标系名称

  // we have a new map, update full size of map
  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  map_received_ = true;
  has_updated_data_ = true;

  // shutdown the map subscrber if firt_map_only_ flag is on
  if (first_map_only_)  // 若first_map_only_开启，第一次接收到地图后，话题将不再接收消息
  {
    ROS_INFO("Shutting down the map subscriber. first_map_only flag is on");
    map_sub_.shutdown();
  }
}

void StaticLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height ; y++)
  {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width ; x++)
    {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }
  x_ = update->x;
  y_ = update->y;
  width_ = update->width;
  height_ = update->height;
  has_updated_data_ = true;
}

void StaticLayer::activate()
{
  onInitialize();
}

void StaticLayer::deactivate()
{
  map_sub_.shutdown();
  if (subscribe_to_updates_)
    map_update_sub_.shutdown();
}

void StaticLayer::reset()
{
  if (first_map_only_)
  {
    has_updated_data_ = true;
  }
  else
  {
    onInitialize();
  }
}

// 更新静态地图边界
void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                               double* max_x, double* max_y)
{
  // 若地图不是窗口滚动的
  if( !layered_costmap_->isRolling() ){
    if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
      // has_extra_bounds_初始值为false，而在第一次结束前，has_updated_data_被设置成false；
      // 且订阅话题在第一次接收到消息后关闭，所以对于static layer，updateBounds只执行一次
      // bounds的范围是整张地图的大小，在updateBounds过程中并没有对静态地图层做任何的更新
      return;
  }

  // 若有extrabounds，就使用
  useExtraBounds(min_x, min_y, max_x, max_y);

  // 将map系中的起点（x_,y_）与终点（x_+width, y_+height）转换到世界系，
  // 确保传入的bound能包含整个map在世界系中的范围
  double wx, wy;

  mapToWorld(x_, y_, wx, wy); // wx、wy是世界坐标系下的原点吧
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;  // 由于默认不更新静态地图层，该层的bound将只更新一次
}

// 这里将更新后的bound传入
// 若不是rolling地图，那么直接将静态地图层bound范围内的内容合并到主地图，因为两者的尺寸也一样
// updateWithTrueOverwrite和updateWithMax采用不同的合并策略，在costmap_layer.cpp中有实现
void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!map_received_)
    return;

  if (!layered_costmap_->isRolling()) // 非滚动
  {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!use_maximum_) // 直接将本层的数据赋值到master map，因为尺寸一样
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    else
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }
  else  // 滚动的，找到静态地图和global系之间的坐标变换，通过主地图->global->静态地图的转换过程，
  {     // 找到主地图的cell在静态地图上对应的cost，赋值给主地图
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    // 首先获得map坐标系相对于global坐标系的位置，这个时候map坐标系随着机器人的运动而运动
    geometry_msgs::TransformStamped transform;
    try
    { // 我设置的map_frame_和global_frame_都是/map
      transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::convert(transform.transform, tf2_transform);
    for (unsigned int i = min_i; i < max_i; ++i)
    {
      for (unsigned int j = min_j; j < max_j; ++j)
      {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        // master map坐标（i，j）->global坐标（wx，wy）
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform*p;
        // global坐标p->静态坐标(mx,my)
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my))
        {
          if (!use_maximum_)  //这里的getCost是继承的Costmap2D中的函数，查询costmap_的mx，my（这两个坐标会转换成一个index在costmap_中查询）的数据
            master_grid.setCost(i, j, getCost(mx, my)); // 其实和上面的两个函数一样，只不过就是转换了一下坐标而已
          else
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
        }
      }
    }
  }
}

}  // namespace costmap_2d
