#ifndef _TRAJ_REPRESENTATION_H
#define _TRAJ_REPRESENTATION_H

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>

#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'

class PathNode {
public:
  /* -------------------- */
  Eigen::Vector2i index;
  int yaw_idx;
  /* --- the state is x y theta(orientation) */
  Eigen::Vector3d state;
  double g_score, f_score;
  double penalty_score;
  /* control input should be steer and arc */
  Eigen::Vector2d input;
  PathNode* parent;
  // 未拓展、在close里面，在open里面三档
  char node_state;
  // 倒车 前进 和速度小于0.01三挡
  int singul = 0;
  /* -------------------- */
  PathNode() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNode(){};
};
typedef PathNode* PathNodePtr;


struct FlatTrajData
{
  int singul;
  std::vector<Eigen::Vector3d> traj_pts;      //某单个方向上均匀采样后的所有坐标点：x,y,t  有终点没有起点
  std::vector<double> thetas; // 储存均匀采样后的yaw
  Eigen::MatrixXd start_state;   //pva
  Eigen::MatrixXd final_state;   // end flat state (2, 3)

};


#endif
