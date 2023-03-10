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

// 代价地图包括静态层、障碍物层、膨胀层。机器人在进行障碍物层的处理之前，已经完成了静态层的地图构建，
// 以及障碍物层和膨胀层的相关准备工作，障碍物层以点云或者激光扫描的形式包含来自传感器的信息，以两个维度跟踪
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// 障碍层插件和名称空间声明
PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstacleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void ObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();  // 得到是否滚动窗口

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  ObstacleLayer::matchSize(); // 根据master map的参数设置障碍物层的宽高、分辨率、原点坐标。value值设为默认
  current_ = true;

  global_frame_ = layered_costmap_->getGlobalFrameID(); // 全局坐标系
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  // 得到要订阅的话题
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);  // 使用字符串流根据空白来分割topics

  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("data_type", data_type, std::string("PointCloud"));
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);

    // 限制传感器数据类型
    if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
    {
      ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
      throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
    }

    std::string raytrace_range_param_name, obstacle_range_param_name;

    // get the obstacle range for the sensor
    // 确定传感器能探测到的障碍物的最大范围值
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // get the raytrace range for the sensor
    // 确定传感器能探测到周围环境的最大范围值
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    // create an observation buffer
    // 创建观察缓冲区push到observation_buffers_中
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance)));

    // check if we'll add this buffer to our marking observation buffers
    if (marking)  // 检查是否要将观察缓冲区的这帧数据插入到标记观察缓冲区中
      marking_buffers_.push_back(observation_buffers_.back());

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing) // 检查是否要将观察缓冲区的这帧数据插入到清除观察缓冲区中
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic
    // 根据不同的数据类型调用不同的回调函数
    if (data_type == "LaserScan")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::LaserScan>
          > sub(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

      boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan> > filter(
        new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, g_nh));

      if (inf_is_valid)
      { // 如果inf有效
        filter->registerCallback(boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1,
                                            observation_buffers_.back()));
        // 回调函数有多个参数：boost::bind(&类名::函数名,类实例指针,参数1,参数2)
        // observation_buffers_.back()的数据是最新的一帧
      }
      else
      { // inf无效
        filter->registerCallback(boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));
      }

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);

      observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
    }
    else if (data_type == "PointCloud")
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

        boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud>
        > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50, g_nh));
        filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }
    else
    {
      boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
          > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

      if (inf_is_valid)
      {
       ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud2>
      > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, g_nh));
      filter->registerCallback(
          boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    }

    if (sensor_frame != "")
    {
      std::vector < std::string > target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);  // 这个是确保两个坐标系存在tf变换？？
    }
  }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);  // 创建动态配置
}

// 动态配置
void ObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
      &ObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

ObstacleLayer::~ObstacleLayer()
{
    if (dsrv_)
        delete dsrv_;
}
// 重配置
void ObstacleLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  combination_method_ = config.combination_method;
}

// 处理缓冲激光扫描消息的回调，允许有inf
void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // project the laser into a point cloud
  // 将激光消息转换成点云
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message->header;

  // project the scan into a point cloud
  try
  { // 进行转换
    projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
             ex.what());
    projector_.projectLaser(*message, cloud);
  }
  catch (std::runtime_error &ex)
  {
    ROS_WARN("transformLaserScanToPointCloud error, it seems the message from laser sensor is malformed. Ignore this laser scan. what(): %s", ex.what());
    return; //ignore this message
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud); //先转换到全局坐标系再存储？ 存储到缓冲区？
  buffer->unlock();
}

// 需要先进行消息过滤，将inf数据转换为range_max的激光扫描消息的回调
void ObstacleLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // Filter positive infinities ("Inf"s) to max_range.
  float epsilon = 0.0001;  // a tenth of a millimeter
  sensor_msgs::LaserScan message = *raw_message;
  for (size_t i = 0; i < message.ranges.size(); i++)
  {
    float range = message.ranges[ i ];
    if ((!std::isfinite(range) && range > 0) || (range == 0.0))
    {
      message.ranges[ i ] = message.range_max - epsilon;
    }
  }

  // project the laser into a point cloud
  sensor_msgs::PointCloud2 cloud;
  cloud.header = message.header;

  // project the scan into a point cloud
  try
  {
    projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
             global_frame_.c_str(), ex.what());
    projector_.projectLaser(message, cloud);
  }
  catch (std::runtime_error &ex)
  {
    ROS_WARN("transformLaserScanToPointCloud error, it seems the message from laser sensor is malformed. Ignore this laser scan. what(): %s", ex.what());
    return; //ignore this message
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud);
  buffer->unlock();
}

void ObstacleLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                               const boost::shared_ptr<ObservationBuffer>& buffer)
{
  sensor_msgs::PointCloud2 cloud2;

  if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
  {
    ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
    return;
  }

  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(cloud2);
  buffer->unlock();
}

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

// 更新边界
// robot_x、robot_y、robot_raw：机器人的x、y坐标值和偏航角
// 用min_x、min_y、max_x、max_y更新边界
void ObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)  // 是否滚动窗口，如果是，更新坐标原点
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  useExtraBounds(min_x, min_y, max_x, max_y); // 确定更新边界轮廓尺寸

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  // 获得用于标记的观测数据
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  // 获得用于清除的观测数据
  current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  // raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    // 清理传感器和障碍物之间的cell
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  // 将新的障碍物放到优先级队列，每个优先级都以零优先级开始
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const sensor_msgs::PointCloud2& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

    for (; iter_x !=iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      double px = *iter_x, py = *iter_y, pz = *iter_z;

      // if the obstacle is too high or too far away from the robot we won't add it
      if (pz > max_obstacle_height_)  //太高或太远就不添加障碍物
      {
        ROS_DEBUG("The point is too high");
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      // 计算从命中点到点云原点的平方距离
      double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
          + (pz - obs.origin_.z) * (pz - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range) // 太远就不需要考虑
      {
        ROS_DEBUG("The point is too far away");
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my))  // 计算需要观测的地图坐标 （从世界坐标->地图坐标）
      {
        ROS_DEBUG("Computing map coords failed");
        continue;
      }

      unsigned int index = getIndex(mx, my);  // 根据地图坐标得到index
      costmap_[index] = LETHAL_OBSTACLE;      // 将障碍物层的相应位置置为障碍物
      // 传入一个bound和一个坐标，若坐标不在bound范围内，它扩张bound，使其包含坐标
      touch(px, py, min_x, min_y, max_x, max_y);  
    }
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

// 更新机器人的位置框
void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      // 判断机器人框的每个点是否在地图内，不在的话就扩充地图大小
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

// 更新代价值
void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (footprint_clearing_enabled_)
  {
    // 这个应该是将全局坐标系下的机器人多边形映射到地图坐标系中，根据映射后的坐标索引设置为自由空间代价
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
    // transformed_footprint_：机器人各个顶点的坐标
  }

  switch (combination_method_)
  {
    case 0:  // Overwrite
      // 覆盖更新（min_i, min_j, max_i, max_j，这个范围内）
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      // 以最大值更新，将主栅格的新值更新为所操作图层和主栅格层值的最大值
      // 如果主地图的代价值是未知状态，就更新为所操作层的代价值
      // 如果所操作层的代价值是未知状态，则主地图层的代价值不变
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

// 添加静态观察
void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

// 清除静态观察
void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

// 获取标记观察
bool ObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations); // 将所有的观察都添加到这个向量里
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  } // 添加静态观察
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

// 获取清除观察
bool ObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  // 获取清除观察
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

// 射线自由空间，清理传感器和障碍物之间的自由空间
void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                              double* max_x, double* max_y)
{
  double ox = clearing_observation.origin_.x; // 获取传感器位置原点ox、oy（global坐标系）
  double oy = clearing_observation.origin_.y;
  const sensor_msgs::PointCloud2 &cloud = *(clearing_observation.cloud_); // 点云

  // get the map coordinates of the origin of the sensor
  unsigned int x0, y0;  // 传感器位置原点（x0、y0是在map坐标系下）
  if (!worldToMap(ox, oy, x0, y0))
  {
    ROS_WARN_THROTTLE(
        1.0, "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy);
    return;
  }

  // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double origin_x = origin_x_, origin_y = origin_y_;  // global坐标系地图原点
  double map_end_x = origin_x + size_x_ * resolution_;  // global坐标系的地图终点
  double map_end_y = origin_y + size_y_ * resolution_;


  touch(ox, oy, min_x, min_y, max_x, max_y);  // 判断ox、oy在不在地图中，不在的话就扩大地图将这个坐标点包含进去

  // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

  // 对点云每个点，我们希望从原点追踪一条线，并清除沿途的障碍物
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
  {
    double wx = *iter_x;  // global坐标系下的点云的一个点
    double wy = *iter_y;

    // 修改
    double inflate_dx = 0.05, inflate_dy = 0.05;
    std::vector<std::pair<double, double>> inflate_pts;
    inflate_pts.emplace_back(std::make_pair(wx + 0, wy + 0));
    inflate_pts.emplace_back(std::make_pair(wx - 0, wy - inflate_dy));
    inflate_pts.emplace_back(std::make_pair(wx - inflate_dx, wy - 0));
    inflate_pts.emplace_back(std::make_pair(wx + 0, wy + inflate_dy));
    inflate_pts.emplace_back(std::make_pair(wx + inflate_dx, wy + 0));
    inflate_pts.emplace_back(std::make_pair(wx - 0, wy - 2 * inflate_dy));
    inflate_pts.emplace_back(std::make_pair(wx - 2 * inflate_dx, wy - 0));
    inflate_pts.emplace_back(std::make_pair(wx + 0, wy + 2 * inflate_dy));
    inflate_pts.emplace_back(std::make_pair(wx + 2 * inflate_dx, wy + 0));
    
    std::vector<std::pair<double, double>>::iterator inflate_iter;
    for(inflate_iter = inflate_pts.begin(); inflate_iter != inflate_pts.end(); inflate_iter++) {

      wx = (*inflate_iter).first;
      wy = (*inflate_iter).second;
      
      double a = wx - ox;
      double b = wy - oy;

      // 获得当前点云的坐标（在世界坐标系进行一个简单的比较计算，如果当前点超过原点/末点范围，通过相似三角形，舍弃掉范围外的部分）
      // the minimum value to raytrace from is the origin
      if (wx < origin_x)
      {
        double t = (origin_x - ox) / a;
        wx = origin_x;
        wy = oy + b * t;
      }
      if (wy < origin_y)
      {
        double t = (origin_y - oy) / b;
        wx = ox + a * t;
        wy = origin_y;
      }

      // the maximum value to raytrace to is the end of the map
      if (wx > map_end_x)
      {
        double t = (map_end_x - ox) / a;
        wx = map_end_x - .001;
        wy = oy + b * t;
      }
      if (wy > map_end_y)
      {
        double t = (map_end_y - oy) / b;
        wx = ox + a * t;
        wy = map_end_y - .001;
      }

      // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
      unsigned int x1, y1;

      // check for legality just in case
      if (!worldToMap(wx, wy, x1, y1))  // 把点云从世界坐标系转换回地图坐标系
        continue;
      
      unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
      MarkCell marker(costmap_, FREE_SPACE);
      // and finally... we can execute our trace to clear obstacles along that line
      // 最后清理该点和传感器原点之间的部分，标记为FREE_SPACE
      raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

      updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    }
    



    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    // 在比例的时候用
    // double a = wx - ox;
    // double b = wy - oy;

    // // 获得当前点云的坐标（在世界坐标系进行一个简单的比较计算，如果当前点超过原点/末点范围，通过相似三角形，舍弃掉范围外的部分）
    // // the minimum value to raytrace from is the origin
    // if (wx < origin_x)
    // {
    //   double t = (origin_x - ox) / a;
    //   wx = origin_x;
    //   wy = oy + b * t;
    // }
    // if (wy < origin_y)
    // {
    //   double t = (origin_y - oy) / b;
    //   wx = ox + a * t;
    //   wy = origin_y;
    // }

    // // the maximum value to raytrace to is the end of the map
    // if (wx > map_end_x)
    // {
    //   double t = (map_end_x - ox) / a;
    //   wx = map_end_x - .001;
    //   wy = oy + b * t;
    // }
    // if (wy > map_end_y)
    // {
    //   double t = (map_end_y - oy) / b;
    //   wx = ox + a * t;
    //   wy = map_end_y - .001;
    // }

    // // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
    // unsigned int x1, y1;

    // // check for legality just in case
    // if (!worldToMap(wx, wy, x1, y1))  // 把点云从世界坐标系转换回地图坐标系
    //   continue;
    
    // unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
    // MarkCell marker(costmap_, FREE_SPACE);
    // // and finally... we can execute our trace to clear obstacles along that line
    // // 最后清理该点和传感器原点之间的部分，标记为FREE_SPACE
    // raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

    // updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
  }
}

// 激活
void ObstacleLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  // 如果停止了，需要重新订阅话题
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();  // 上次更新时间置为现在
  }
}
// 停用
void ObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe(); // 不再订阅
  }
}

// 更新跟踪边界
void ObstacleLayer::updateRaytraceBounds(double ox, double oy, double wx, double wy, double range,
                                         double* min_x, double* min_y, double* max_x, double* max_y)
{
  double dx = wx-ox, dy = wy-oy;
  double full_distance = hypot(dx, dy);
  double scale = std::min(1.0, range / full_distance);
  double ex = ox + dx * scale, ey = oy + dy * scale;
  touch(ex, ey, min_x, min_y, max_x, max_y);
}

// 重置
void ObstacleLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

}  // namespace costmap_2d
