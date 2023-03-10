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
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

// 计算规划代价函数
// costs是地图指针，内容是地图cell的代价值，cycles为循环次数，代码里设置的是nx*ny*2为地图栅格数的二倍
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear(); // queue_是个容器，vector<Index>， Index包括索引值i和栅格点的cost
    int start_i = toIndex(start_x, start_y);    // 计算开始点的栅格索引 
    queue_.push_back(Index(start_i, 0));
    // ns_是地图的栅格数，ns_ = nx_ * ny_
    std::fill(potential, potential + ns_, POT_HIGH);    // 就是向potential里分配ns_个空间，并赋值为POT_HIGH
    potential[start_i] = 0; // 起始点的代价为0；

    int goal_i = toIndex(end_x, end_y); // 得到终点的索引值
    int cycle = 0;  // 初始化cycle为0，就是最多找cycles(2 * nx_ * ny_)次

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];  // 最小代价点
        // 把容器最小的一个元素移动到最后，对其他元素进行从小到大排序
        std::pop_heap(queue_.begin(), queue_.end(), greater1()); 
        queue_.pop_back();  // 删除最小代价的点

        int i = top.i;
        if (i == goal_i)    // 判断是否是目标点，如果是就退出
            return true;
        // 将代价最小的点i周围四个（上下点就是i+nx和i-nx）点加入搜索队里并更新代价值
        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;    // 循环次数加1
    }

    return false;   // 如果循环完了还没找到终点，就返回false
}

// 添加点并更新相应的代价值
void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    if (next_i < 0 || next_i >= ns_)    // 判断是否超出范围
        return;

    if (potential[next_i] < POT_HIGH)   // 代表这个点已经搜索过
        return;
    // 这个点是障碍物
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    // p_calc_->calculatePotential有两种方法计算
    // 采用简单方法计算cost[next_i] + neutral_cost + prev_potential
    // 地图代价 + 单格距离代价（初始化为50）（因为从i->next_i移动了一个单元格）+ 之前路径代价（G）
    // 复杂方法需要进行二次曲线计算，默认是这个
    // potential里放的都是移动到这个点需要的代价，而不包括这个点移动到终点需要的代价，
    // queue_中包含了所有的代价值，用来进行比较
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    // 找到next_i的坐标点x、y
    int x = next_i % nx_, y = next_i / nx_;
    // 找到next_i到终点的距离（默认只能上下左右移动）
    float distance = abs(end_x - x) + abs(end_y - y);
    // potential[next_i] + distance * neutral_cost_ 为总代价F = G + H， H为到目标点距离的代价
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    // 插入数据后进行排序，将代价值最小的那个放到最前面
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace global_planner
