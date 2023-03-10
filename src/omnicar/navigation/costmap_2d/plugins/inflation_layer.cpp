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
#include <algorithm>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <boost/thread.hpp>
#include <pluginlib/class_list_macros.h>

// InflationLayer没有自身的栅格地图要维护，直接在master地图上进行操作，它根据膨胀参数设置用来膨胀的“参考”矩阵
// 并在master地图上从障碍物出发，不断传播更新，完成对整个地图障碍的膨胀，等效于完成一个由“将机器人视为一个点”到“考虑机器人本身体积”的转变过程
// 避免因为忽视了足迹而碰上障碍

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

InflationLayer::InflationLayer()
  : resolution_(0)
  , inflation_radius_(0)
  , inscribed_radius_(0)
  , weight_(0)  // cost_scaling_factor
  , inflate_unknown_(false)
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , dsrv_(NULL)
  , seen_(NULL)
  , cached_costs_(NULL)
  , cached_distances_(NULL)
  , last_min_x_(-std::numeric_limits<float>::max())
  , last_min_y_(-std::numeric_limits<float>::max())
  , last_max_x_(std::numeric_limits<float>::max())
  , last_max_y_(std::numeric_limits<float>::max())
{
  inflation_access_ = new boost::recursive_mutex();
}

// 初始化工作，最主要的是调用了matchSize函数
void InflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    need_reinflation_ = false;
    // 动态改参
    dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb = boost::bind(
        &InflationLayer::reconfigureCB, this, _1, _2);

    if (dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
  }
  // 由于InflationLayer没有继承Costmap2D，所以它和静态地图与障碍物地图这两层不同，没有自己这一层的栅格地图要维护
  // 所以matchSize函数不需要根据master地图的参数来调节本层参数。这个函数先获取著地图的分辨率，接着调用cellDistance函数
  // 这个函数可以把global坐标系以米为单位的长度转换成以cell为单位的距离，所以可以获得地图上的膨胀参数cell_inflation_radius_
  matchSize();
}

void InflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
{
  setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

  if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown) {
    enabled_ = config.enabled;
    inflate_unknown_ = config.inflate_unknown;
    need_reinflation_ = true;
  }
}

void InflationLayer::matchSize()
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_); // 获得cell_inflation_radius，也就是膨胀几个cell
  computeCaches();  // 完成两个参考矩阵的填充

  // 根据主地图的大小创建seen_数组，它用于标记cell是否已经计算过
  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_)
    delete[] seen_;
  seen_size_ = size_x * size_y;
  seen_ = new bool[seen_size_];
}

// 更新膨胀地图边界
void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (need_reinflation_)  // 默认是false
  {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
  else
  {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y; // 膨胀层在传入bound的值的基础上，通过inflation_radius_再次扩张
    *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
    *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
    *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
    *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
  }
}

void InflationLayer::onFootprintChanged()
{
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();
  need_reinflation_ = true;

  ROS_DEBUG("InflationLayer::onFootprintChanged(): num footprint points: %lu,"
            " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
            layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

// 更新膨胀地图代价
void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  if (cell_inflation_radius_ == 0)
    return;

  // make sure the inflation list is empty at the beginning of the cycle (should always be true)
  ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");

  // 通过指针master_array指向master map，确认seen_数组被正确设置
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_ == NULL) {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_]; // 分配和master map一样大小的数组
  }
  else if (seen_size_ != size_x * size_y)
  {
    ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
    delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
  }
  memset(seen_, false, size_x * size_y * sizeof(bool));

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  // 边界膨胀
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  // 限制范围
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(int(size_x), max_i);
  max_j = std::min(int(size_y), max_j);

  // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
  // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

  // Start with lethal obstacles: by definition distance is 0.0
  // 接下来遍历bound中的cell，找到cost为LETHAL_OBSTACLE,即障碍物的cell，将其以CellData形式放进inflation_cell_[0.0]中
  // inflation_cells_定义如下：std::map<double, std::vector> inflation_cells_
  // 距离为0的即障碍物本身，目前inflation_cells_只包含障碍物本身
  std::vector<CellData>& obs_bin = inflation_cells_[0.0];
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE)
      {
        obs_bin.push_back(CellData(index, i, j, i, j)); // 后面两个参数是距离这个cell最近的障碍物的坐标，因为他们本身就是障碍物，所以传入自身的坐标
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  // 遍历，外层遍历键，即距离障碍物的距离，内层遍历‘值’，即cell，（第一层遍历的时候，键值只有0.0，后面会添加障碍物周围的cell，逐渐外扩）
  std::map<double, std::vector<CellData> >::iterator bin;
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
  {
    for (int i = 0; i < bin->second.size(); ++i)
    {
      // process all cells at distance dist_bin.first
      const CellData& cell = bin->second[i];  // cell点

      unsigned int index = cell.index_; // 记录该cell的索引

      // ignore if already visited
      if (seen_[index])         // 判断这个cell有没有被遍历过
      {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;    // 这两个坐标是距离这个cell最近的障碍物的坐标
      unsigned int sy = cell.src_y_;

      // assign the cost associated with the distance from an obstacle to the cell
      // 根据cell点坐标和距离这个cell最近的障碍物的坐标，获得cost，就是算出来距离，然后查cached_costs_这个事先计算好的表
      unsigned char cost = costLookup(mx, my, sx, sy);  
      unsigned char old_cost = master_array[index]; // 获得该cell的master map的原cost值
      // 如果原cost值为NO_INFORMATION： 
      //                              （1）inflate_unknown_为true：当新的cost > 0，设为新的cost
      //                              （2）inflate_unknown_为false：新的cost >= INSCRIBED_INFLATED_OBSTACLE,设为新的cost
      // 区别在于：如果inflate_unknown_，则当膨胀到主地图的未知区域，只要有cost，就覆盖它；
      // 而当inflate_unknown_关闭，当膨胀到主地图上的未知区域，只有新cost是障碍物，才覆盖它，否则维持未知。后者膨胀的范围更窄
      if (old_cost == NO_INFORMATION && (inflate_unknown_ ? (cost > FREE_SPACE) : (cost >= INSCRIBED_INFLATED_OBSTACLE)))
        master_array[index] = cost;
      else  // 否则，取cost最大的来更新
        master_array[index] = std::max(old_cost, cost);

      // attempt to put the neighbors of the current cell onto the inflation list
      // 接下来enqueue函数会将其四周的cell按照距离远近加入inflation_cells_对应的键下
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy);
    }
  }

  inflation_cells_.clear();
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
// index是要插入cell的索引，mx、my是要插入的cell的坐标，src_x,src_y是障碍物坐标（这里应该是距离（mx,my）最近的障碍物坐标）
inline void InflationLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                                    unsigned int src_x, unsigned int src_y)
{
  if (!seen_[index])
  {
    // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    // 在cached_distances_上找到当前cell与最近的障碍物的距离
    double distance = distanceLookup(mx, my, src_x, src_y);

    // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
    // 当距离不超过阈值，就存放到inflation_cells_数组中，否则直接返回
    if (distance > cell_inflation_radius_)
      return;

    // push the cell data onto the inflation list and mark
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
  }
}

// 参考矩阵， 这里称cached_costs_和cached_distances_为参考矩阵，因为他们是后续膨胀计算的参考物
void InflationLayer::computeCaches()
{
  if (cell_inflation_radius_ == 0)  // 由于是根据cell_inflation_radius_来设置参考矩阵的，所以为0时直接返回
    return;

  // based on the inflation radius... compute distance and cost caches
  // 这里是来初始化两个参考矩阵的，只在第一次进入时执行
  if (cell_inflation_radius_ != cached_cell_inflation_radius_)
  {
    deleteKernels();  // 如果cached_distances_和cached_costs_不为null，先清空
    // 设置大小两个矩阵都是（cell_inflation_radius_ + 2）*（cell_inflation_radius_ + 2）
    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        // 为什么要距离（0，0）呢？？？
        cached_distances_[i][j] = hypot(i, j);  // 设置cached_distances_矩阵的元素值为每个元素到（0，0）点的三角距离
      }
    }
    // 保证第二次进函数的时候不进if里
    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
  {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
    {
      // 最后调用computeCost将cached_distances_转换成cached_costs_
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
    }
  }
}

void InflationLayer::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  if (cached_costs_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_costs_[i])
        delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
    cached_costs_ = NULL;
  }
}

void InflationLayer::setInflationParameters(double inflation_radius, double cost_scaling_factor)
{
  if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
  {
    // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
    // when accessing the cached arrays
    boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);

    inflation_radius_ = inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }
}

}  // namespace costmap_2d
