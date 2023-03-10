/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: KD tree functions
 * Author: Andrew Howard
 * Date: 18 Dec 2002
 * CVS: $Id: pf_kdtree.h 6532 2008-06-11 02:45:56Z gbiggs $
 *************************************************************************/

#ifndef PF_KDTREE_H
#define PF_KDTREE_H

#ifdef INCLUDE_RTKGUI
#include "rtk.h"
#endif

/*
  将pose和权重保存在一个kdtree中，加速pose的查找和一系列计算。kdtree是二叉搜索树的变种
  粒子滤波中的粒子就是pose+权重
*/


// Info for a node in the tree
// kdtree的基本单元是node
typedef struct pf_kdtree_node
{
  // Depth in the tree
  // depth是树的深度，leaf是代表什么
  int leaf, depth;

  // Pivot dimension and value
  int pivot_dim;  //比较哪一维？？？
  double pivot_value;

  // The key for this node，x,y,theta
  int key[3]; //pose，只不过等比例放大了点，存成int了

  // The value for this node
  double value; // 某个节点的权重，这个权重就是判断这个点的坐标是不是机器人的位置权重大小

  // The cluster label (leaf nodes)
  int cluster;  // 集群？？？

  // Child nodes  两个孩子（左右两个节点，和树的一样）
  struct pf_kdtree_node *children[2]; 

} pf_kdtree_node_t;
// 这里的kdtree多维护了一个cluster，如果自己有关键数据也可以在这儿添加，并添加函数维护

// A kd tree
typedef struct
{
  // Cell size
  // 放大pose的比例，默认是0.5，0.5，pi/180*10，除一下就放大了
  double size[3];

  // The root node of the tree
  pf_kdtree_node_t *root; // 根节点？

  // The number of nodes in the tree
  int node_count, node_max_count; // 这个应该是当前node的数量和最大node的数量
  pf_kdtree_node_t *nodes;  // ketree在一开始建立的时候就会malloc个内存存储node，

  // The number of leaf nodes in the tree
  int leaf_count; // 叶子的数量

} pf_kdtree_t;


// Create a tree
extern pf_kdtree_t *pf_kdtree_alloc(int max_size);

// Destroy a tree
extern void pf_kdtree_free(pf_kdtree_t *self);

// Clear all entries from the tree
extern void pf_kdtree_clear(pf_kdtree_t *self);

// Insert a pose into the tree
extern void pf_kdtree_insert(pf_kdtree_t *self, pf_vector_t pose, double value);

// Cluster the leaves in the tree
extern void pf_kdtree_cluster(pf_kdtree_t *self);

// Determine the probability estimate for the given pose
extern double pf_kdtree_get_prob(pf_kdtree_t *self, pf_vector_t pose);

// Determine the cluster label for the given pose
extern int pf_kdtree_get_cluster(pf_kdtree_t *self, pf_vector_t pose);


#ifdef INCLUDE_RTKGUI

// Draw the tree
extern void pf_kdtree_draw(pf_kdtree_t *self, rtk_fig_t *fig);

#endif

#endif
