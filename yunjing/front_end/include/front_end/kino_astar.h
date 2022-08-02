#ifndef _KINO_ASTAR_H_
#define _KINO_ASTAR_H_

#include "model_gazebo/world2oct.h"
#include "traj_representation.h"

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <queue>


#define inf 1 >> 30

// 用于比较f
class NodeComparator {
public:
  template <class NodePtr>
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

// 用于寻找点
template <typename T>
  struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
      size_t seed = 0;
      for (long int i = 0; i < matrix.size(); ++i) {
        auto elem = *(matrix.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };


template <class NodePtr>
  class NodeHashTable {
  private:
    /* data */

    std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash<Eigen::Vector2i>> data_2d_;
    std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>> data_3d_;
    
  public:
    NodeHashTable(/* args */) {
    }
    ~NodeHashTable() {
    }
    // : for 2d vehicle planning
    void insert(Eigen::Vector2i idx, NodePtr node) {
      data_2d_.insert(std::make_pair(idx, node));
    }
    //for 3d vehicle planning
    void insert(Eigen::Vector2i idx, int yaw_idx, NodePtr node) {
      data_3d_.insert(std::make_pair(Eigen::Vector3i(idx(0), idx(1), yaw_idx), node));
    }
    void insert(Eigen::Vector3i idx,NodePtr node ){
      data_3d_.insert(std::make_pair(idx,node));
    }

    NodePtr find(Eigen::Vector2i idx) {
      auto iter = data_2d_.find(idx);
      return iter == data_2d_.end() ? NULL : iter->second;
    }
    NodePtr find(Eigen::Vector2i idx, int yaw_idx) {
      auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), yaw_idx));
      return iter == data_3d_.end() ? NULL : iter->second;
    }

    void clear() {
      data_2d_.clear();
      data_3d_.clear();
    }
  };


class KinoAstar{
  private:
    ros::NodeHandle nh_;

    // 全部格子
    std::vector<PathNodePtr> path_node_pool_;
    // 哈希搜索拓展过的节点
    NodeHashTable<PathNodePtr> expanded_nodes_;
    // OPEN集
    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set_;
   
    // Eigen::Vector2d horizon_; // 搜索范围
    double yaw_resolution_; // yaw角离散间隔
    double inv_yaw_resolution_;
    double lambda_heu_; // 贪婪搜索的系数
    int allocate_num_; // 全部的格子数
    double max_search_time_; // 最大搜索时间
    double traj_forward_penalty_; // 向前进惩罚系数
    double traj_back_penalty_; // 前后惩罚系数
    double traj_gear_switch_penalty_; //前进倒车状态切换惩罚
    double traj_steer_penalty_; // 转角惩罚
    double traj_steer_change_penalty_; // 转角改变惩罚
    double step_arc_; // 前向搜索位移
    double checkl_; // oneshot检查中间点碰撞离散间隔
    double grid_interval_;
    int check_num_; // 用于A*拓展状态时检查碰撞
    double oneshot_range_;
    double max_kinoastar_dis_;//前端给出的最大距离
    double sampletime_;//给到后端时的采样时间

    double tread_;
    double front_suspension_;
    double rear_suspension_;
    double base_height_;

    //地图范围
    double x_upper, y_upper, z_upper;
	  double x_lower, y_lower, z_lower;
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

    int use_node_num_;//统计用了多少节点
    int iter_num_;

    Eigen::Vector3d map_size_3d_;
    
    double yaw_origin_; //用于将yaw转成index
    double non_siguav_; //不能小于此速度
    double Collision_interval_;//碰撞检测间隔
    
    // 注释掉的参数是原文有但是我没用
    double max_vel_;
    double max_acc_;
    double max_cur_;
    double max_steer_;
    double astar_cur_;
    // double resolution_;
    double length_;
    double width_;
    double height_;
    double wheel_base_;

    double stable_z_;

    // Eiegn::vector3d origin_;

    // ompl 此实现显式计算所有 48 条 Reeds-Shepp 曲线并返回最短的有效 解决方案。括号里是旋转半径
    ompl::base::StateSpacePtr shotptr_;


    // 过程变量
    bool is_shot_succ_= false;

    std::vector<double> shot_lengthList;// 那些段的距离
    std::vector<double> shot_timeList;// 梯形速度计算的时间
    std::vector<int> shotindex;// 转折点在数组的坐标
    std::vector<int> shot_SList;// 存储每一段的方向



    ros::Publisher expandNodes_pub_;
  public:
    std::shared_ptr<World2oct> map_;    
    // 终点路径
    std::vector<PathNodePtr> path_nodes_;
    std::vector<PathNodePtr> kino_nodes_;
    std::vector<FlatTrajData> flat_trajs_;
    std::vector<Eigen::Vector3d> sample_trajs_;
    Eigen::Vector4d start_state_;// x y yaw 方向
    Eigen::Vector2d start_ctrl_;
    Eigen::Vector4d end_state_;
    double totalTrajTime;
    bool has_path_ = false;

    
    enum { REACH_HORIZON = 1, REACH_END = 2,  NO_PATH = 3, REACH_END_BUT_SHOT_FAILS = 4};
    KinoAstar(ros::NodeHandle nh,std::shared_ptr<World2oct> map){
      nh_ = nh;
      map_.reset(map.get());
    }

    ~KinoAstar(){
      for (int i = 0; i < allocate_num_; i++){
        delete path_node_pool_[i];
      }
    }
    void init();
    void reset();


    int search(const Eigen::Vector4d &end_state, Eigen::Vector4d &start_state, Eigen::Vector2d &init_ctrl);
    bool IfCollision_PosYaw(const Eigen::Vector2d &pos, const double &yaw);
    int yawToIndex(const double &yaw);
    double normalize_angle(const double &angle);
    inline int getSingularity(const double &vel);
    inline double getHeu(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2);
    bool is_shot_sucess(const Eigen::Vector3d &state1, const Eigen::Vector3d &state2);
    double computeShotTraj(const Eigen::Vector3d &state1, const Eigen::Vector3d &state2,
                           std::vector<Eigen::Vector3d> &path_list,
                           double& len);
    void retrievePath(const PathNodePtr &end_node);
    void stateTransit(const Eigen::Vector3d &state0, const Eigen::Vector2d &ctrl_input, Eigen::Vector3d &state1);


    void NodeVis(const Eigen::Vector3d &state);
    void getKinoNode();
    double evaluateDuration(const double &length, const double &startV, const double &endV);
    double evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV);
    void getFlatState(const Eigen::Vector4d &state, const Eigen::Vector2d &control_input,const int &singul,Eigen::MatrixXd &flat_state);
    
    Eigen::Vector3d evaluatePos(const double &t);

    bool IfCollision_Toward(const Eigen::Vector2d &pos, const double &yaw);

};


#endif