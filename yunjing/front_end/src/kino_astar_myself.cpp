#include "front_end/kino_astar.h"


void KinoAstar::init(){
  std::string node_name = ros::this_node::getName();
  nh_.param<double>(ros::this_node::getName()+ "/yaw_resolution",yaw_resolution_,0.3);
  nh_.param<double>(ros::this_node::getName()+ "/lambda_heu",lambda_heu_,5.0);
  nh_.param<double>(ros::this_node::getName()+ "/max_seach_time",max_search_time_,0.1);
  nh_.param<double>(ros::this_node::getName()+ "/traj_forward_penalty",traj_forward_penalty_,1.0);
  nh_.param<double>(ros::this_node::getName()+ "/traj_back_penalty",traj_back_penalty_,2.5);
  nh_.param<double>(ros::this_node::getName()+ "/traj_gear_switch_penalty",traj_gear_switch_penalty_,15.0);
  nh_.param<double>(ros::this_node::getName()+ "/traj_steer_penalty",traj_steer_penalty_,0.5);
  nh_.param<double>(ros::this_node::getName()+ "/traj_steer_change_penalty",traj_steer_change_penalty_,0);
  nh_.param<double>(ros::this_node::getName()+ "/traj_toword_occ_penalty",traj_toword_occ_penalty_,5);
  nh_.param<double>(ros::this_node::getName()+ "/step_arc",step_arc_,0.9);
  nh_.param<double>(ros::this_node::getName()+ "/checkl",checkl_,0.2);
  inv_yaw_resolution_ = 1/yaw_resolution_;
  grid_interval_ = map_->grid_interval_;
  nh_.param<double>(ros::this_node::getName()+ "/stable_z",stable_z_,0.2);
  nh_.param<int>(ros::this_node::getName()+ "/check_num",check_num_,5);
  nh_.param<double>(ros::this_node::getName()+ "/oneshot_range",oneshot_range_,5);
  nh_.param<double>(ros::this_node::getName()+ "/max_kinoastar_dis",max_kinoastar_dis_,10);
  nh_.param<double>(ros::this_node::getName()+ "/sampletime",sampletime_,0.1);
  nh_.param<double>(ros::this_node::getName()+ "/step_diff",step_diff_,0.2);
  nh_.param<double>(ros::this_node::getName()+ "/traj_toword_occ_scope",traj_toword_occ_scope_,0.8);


  
// ROS_INFO("\033[41;37m stable_z_:%f \033[0m",stable_z_);  
  x_upper = map_->x_upper;
  y_upper = map_->y_upper;
  z_upper = map_->z_upper;
  x_lower = map_->x_lower;
  y_lower = map_->y_lower;
  z_lower = map_->z_lower;
  GLX_SIZE = map_->GLX_SIZE;
  GLY_SIZE = map_->GLY_SIZE;
  GLZ_SIZE = map_->GLZ_SIZE;
  allocate_num_ = map_->GLX_SIZE*GLY_SIZE;
// ROS_INFO("\033[43;37m x_upper:%f x_lower:%f y_upper:%f y_lower:%f z_upper:%f z_lower:%f \033[0m",x_upper,x_lower,y_upper,y_lower,z_upper,z_lower);

  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  map_size_3d_(0) = x_upper - x_lower;
  map_size_3d_(1) = y_upper - y_lower;
  map_size_3d_(2) = z_upper - z_lower;
  yaw_origin_ = -M_PI;
  
  nh_.param<double>(ros::this_node::getName()+ "/max_vel",max_vel_,5);
  nh_.param<double>(ros::this_node::getName()+ "/max_acc",max_acc_,5);
  nh_.param<double>(ros::this_node::getName()+ "/max_cur",max_cur_,0.5);
  nh_.param<double>(ros::this_node::getName()+ "/length",length_,1);
  nh_.param<double>(ros::this_node::getName()+ "/width",width_,1);
  nh_.param<double>(ros::this_node::getName()+ "/height",height_,0.5);
  nh_.param<double>(ros::this_node::getName()+ "/tread",tread_,0.3);
  nh_.param<double>(ros::this_node::getName()+ "/front_suspension",front_suspension_,0.05);
  nh_.param<double>(ros::this_node::getName()+ "/rear_suspension",rear_suspension_,0.05);
  nh_.param<double>(ros::this_node::getName()+ "/wheel_base",wheel_base_,0.3);
  nh_.param<double>(ros::this_node::getName()+ "/base_height",base_height_,0.2);

  nh_.param<double>(ros::this_node::getName()+ "/non_siguav",non_siguav_,0.01);
  nh_.param<double>(ros::this_node::getName()+ "/Collision_interval",Collision_interval_,0.1);

  
  
  max_steer_ = std::atan(wheel_base_*max_cur_);
  
  
// ROS_INFO("\033[41;37m path:%s width_:%f \033[0m",(ros::this_node::getName()+ "width").c_str(),width_);  
  shotptr_ =std::make_shared<ompl::base::ReedsSheppStateSpace>(1.0/max_cur_);

  expandNodes_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("kinoastar/expanded_nodes",1);
}

void KinoAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();
  kino_nodes_.clear();
  flat_trajs_.clear();
  sample_trajs_.clear();
  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

int KinoAstar::search(const Eigen::Vector4d &end_state, Eigen::Vector4d &start_state, Eigen::Vector2d &init_ctrl){
  // initsearch用于判断是否需要考虑初始状态有速度，false表示初始位置仅能向前搜索
  bool isocc = false;  bool initsearch = false;
  // 计算运行时间
  double start_time = ros::Time::now().toSec();
  // 检查初末状态是否发生碰撞  用一个地图类来实现一下
  isocc = IfCollision_PosYaw(start_state.head(2),start_state[2]);
  if(isocc){
    ROS_ERROR("KinoAstar: head is not free!");
    return NO_PATH;
  }
  isocc = IfCollision_PosYaw(end_state.head(2),end_state[2]);
  if(isocc){
    ROS_WARN("KinoAstar: end is not free!");
    return NO_PATH;
  }
  start_state_ = start_state;
  start_ctrl_ = init_ctrl;
  end_state_ = end_state;
  Eigen::Vector2i end_index = map_->coord2grid2dIndex(end_state.head(2));

  // 用起点初始化path_node_pool_
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state = start_state.head(3);
  cur_node->index = map_->coord2grid2dIndex(start_state.head(2));
  cur_node->yaw_idx = yawToIndex(start_state[2]);
  cur_node->g_score = 0.0;
  cur_node->input = Eigen::Vector2d(0.0,0.0);
  cur_node->singul = getSingularity(start_state(3));
  cur_node->f_score = lambda_heu_ * getHeu(cur_node->state, end_state);
  cur_node->node_state = IN_OPEN_SET;
  // 将起点加到OPen中
  open_set_.push(cur_node);
  use_node_num_ += 1;
  // 维护拓展过的节点的数组
  expanded_nodes_.insert(cur_node->index, yawToIndex(start_state[2]),cur_node);
  PathNodePtr terminate_node = NULL;
  // 如果初始速度小于0的话就不用考虑方向了  否则需要考虑方向
  if(cur_node->singul == 0){ initsearch = true;}
  while (!open_set_.empty()){
    // ROS_INFO("in open_set_!");
    cur_node = open_set_.top();
    // to decide the near end ？？不知道reach_horizon有啥用 好像后面也没用？？
    // bool reach_horizon = (cur_node->state.head(2) - start_state_.head(2)).norm() >= horizon_;
    if((cur_node->state.head(2) - end_state_.head(2)).norm()<oneshot_range_ && initsearch){
      ROS_WARN("one shotting!");
      is_shot_sucess(cur_node->state,end_state_.head(3));
    }

    if (is_shot_succ_){
      terminate_node = cur_node;
      // //////////////////////////////////////////////////////////////
      start_state = end_state;
      init_ctrl = Eigen::Vector2d(0.0, 0.0);
      // //////////////////////////////////////////////////////////////
      retrievePath(terminate_node);
      has_path_ = true;
      ROS_WARN("one shot! iter num: %d",iter_num_);
      return REACH_END;
    }
// ROS_INFO("\033[43;37m shot_fail \033[0m");
    // 如果one_shot失败 且时间超过了规定时间则把这个点作为终点
    // 计算当前运行时间
    double runTime = ros::Time::now().toSec()-start_time;
    // double runTime = (std::chrono::system_clock::now()-start_time).count();
// ROS_INFO("\033[43;37m runTime:%f max_search_time_:%f \033[0m",runTime,max_search_time_);
    if(runTime > max_search_time_){
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;
      // //////////////////////////////////////////////////////////////
      start_state.head(3) = terminate_node->state;
      start_state[3] = terminate_node->singul;
      init_ctrl = terminate_node->input;
      // //////////////////////////////////////////////////////////////
      if (terminate_node->parent == NULL){
        std::cout << "[34mKino Astar]: terminate_node->parent == NULL" << std::endl;
        printf("\033[Kino Astar]: NO_PATH \n\033[0m");
        return NO_PATH;
      }
      else{
        ROS_WARN("KinoSearch: Reach the max seach time");
        return REACH_END;
      }
    }
    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;
    // 利用采样获得输入量
    Eigen::Vector3d cur_state = cur_node->state;
    Eigen::Vector3d pro_state;
    Eigen::Vector2d ctrl_input;//sl,sr
    std::vector<Eigen::Vector2d> inputs;
    // !initsearch时其实可以搜索的更精细一些
    // 前向搜索策略：
    // 两轮速度比为2:-1转pi/4执行高曲率转角
    // 两轮速度比为1：1，2：1，2：0分别执行固定距离（或者不固定）前向搜索
    double const_arc_length = 0.2;
    if(!initsearch){
      if(start_state_[3]>0){
        for(int sl = -1; sl <= 2; sl++){
          for(int sr = -1; sr <= 2; sr++){
            if( sl+sr <=0 )
              continue;
            ctrl_input<< sl*step_diff_, sr*step_diff_;
            inputs.push_back(ctrl_input);
          }
        }
      }
      else{
        for(int sl = -2; sl <= 1; sl++){
          for(int sr = -2; sr <= 1; sr++){
            if( sl+sr >=0 )
              continue;
            ctrl_input<< sl*step_diff_, sr*step_diff_;
            inputs.push_back(ctrl_input);
          }
        } 
      }
      initsearch = true;
    }
    // else{
    //   for (int sl = -2; sl <= 2; sl++){
    //     for(int sr = -2; sr <= 2; sr++){
    //       if( sl+sr==0 &&(sl==-2 || sl==2 || sl==0))
    //         continue;
    //       ctrl_input<< sl*step_diff_, sr*step_diff_;
    //       inputs.push_back(ctrl_input);
    //     }
    //   }
    // }
    else{
      for (int sl = -2; sl <= 2; sl++){
        for(int sr = -2; sr <= 2; sr++){
          // if( sl+sr==0 &&(sl==-2 || sl==2 || sl==0))
          if( sl+sr==0 )
            continue;
          else if( fabs(sl-sr)!=3 ){
            double current_ac_length = (sl+sr)*step_diff_;
            double sgn = current_ac_length/abs(current_ac_length);
            ctrl_input<< sl*step_diff_*const_arc_length/current_ac_length, sr*step_diff_*const_arc_length/current_ac_length;
          }
          else{
            double current_ac_length = (sl+sr)*step_diff_;
            double real_ac_length = tread_ * M_PI / 4 / 3;
            ctrl_input<< sl*step_diff_*real_ac_length/current_ac_length, sr*step_diff_*real_ac_length/current_ac_length;
          }
          inputs.push_back(ctrl_input);
        }
      }
    }




    for (auto& input:inputs){
      // std::cout<<"input: "<<input[0]<<"  "<<input[1]<<std::endl;
      int singul;
      if(fabs(input[0]+input[1])<1e-3)
        singul = 0;
      else
        singul = (input[0]+input[1])>0?1:-1;
      // 获得该采样之后的状态 pro_state
      stateTransit(cur_state, input, pro_state);


      /* inside map range *///检查pro_state是不是在地图里面
      if (pro_state(0) <= x_lower || pro_state(0) >= x_upper ||
          pro_state(1) <= y_lower || pro_state(1) >= y_upper){
        std::cout << "[Kino Astar]: out of map range" << std::endl;
        continue;
      }
      //检查pro_state是否在close set里面，如果在的话就跳过
      // pro_state所在的方格
      Eigen::Vector2i pro_id = map_->coord2grid2dIndex(pro_state.head(2));
      double pro_yaw_id = yawToIndex(pro_state[2]);
      PathNodePtr pro_node;
        pro_node = expanded_nodes_.find(pro_id, pro_yaw_id);
      if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET){
        continue;
      }

      // 检查pro_state和cur_state在不在一个方格内  是的话不要这个控制量
      Eigen::Vector2i diff = pro_id - cur_node->index;
      int diff_yaw = pro_yaw_id - cur_node->yaw_idx;
      /* collision free */
      Eigen::Vector3d xt;

      // 将轨迹长度分为check_num_份进行碰撞检测
      for (int k = 1; k <= check_num_; ++k){
        // 对轨迹长度进行采样  转向角不变
        double vl = input[0] * double(k) / double(check_num_);
        double vr = input[1] * double(k) / double(check_num_);
        Eigen::Vector2d tmpctrl; tmpctrl << vl,vr;
        // 计算采样后的点
        stateTransit(cur_state, tmpctrl, xt);
        // 按照一定的间隔检测汽车周围点是否发生碰撞
        isocc = IfCollision_PosYaw(xt.head(2),xt[2]);
        if (isocc){
          // ROS_WARN("occcccccccccccccccccccccccccccccccccc!");
          break;
        }
      }
      if (isocc)  continue;
      NodeVis(pro_state);

      /* ---------- compute cost ---------- */
      double tmp_g_score = 0.0;
      double tmp_f_score = 0.0;
      double real_g_score = 0.0;
      int lastDir = cur_node->singul;
      // 前后向的惩罚不同
          // ROS_INFO("\033[40;36m getting traj_pts!!  shot_lengthList i: %d\033[0m",i); 
      if(singul>0){
        real_g_score +=  std::fabs(input[0]+input[1]) * traj_forward_penalty_;
      }
      else{
        real_g_score += std::fabs(input[0]+input[1]) * traj_back_penalty_;
      }
      // std::cout<<"1111111111111111gscore: "<<tmp_g_score<<"\n";
      // singul是当前拓展输入的方向  如果与当前所在点的方向不同，则加惩罚
      if(singul * lastDir < 0){
        tmp_g_score += traj_gear_switch_penalty_;
      }
      // 转向惩罚，多走直线
      // tmp_g_score += traj_steer_penalty_ * std::fabs(input[0]) * std::fabs(input[1]);
      tmp_g_score += traj_steer_penalty_ * std::fabs(input[0]-input[1]);
      // 变转角惩罚，尽量跟上一个点一个转角
      // tmp_g_score += traj_steer_change_penalty_ * std::fabs(input[0]-cur_node->input[0]);
      // tmp_g_score += cur_node->g_score;
      tmp_g_score += traj_steer_change_penalty_ * std::fabs(input[0]-cur_node->input[0]);
      
      // 不准冲着障碍物去的惩罚
      if(IfCollision_Toward(cur_node->state.head(2),cur_node->state[2]))
        tmp_g_score += traj_toword_occ_penalty_;

      real_g_score += cur_node->g_score;

      tmp_f_score = tmp_g_score + real_g_score + lambda_heu_ * getHeu(pro_state, end_state);

      // 一个没有拓展过的节点，新增
      if (pro_node == NULL){
        pro_node = path_node_pool_[use_node_num_];
        pro_node->index = pro_id;
        pro_node->state = pro_state;
        pro_node->yaw_idx = pro_yaw_id;
        pro_node->f_score = tmp_f_score;
        pro_node->g_score = real_g_score;
        pro_node->penalty_score = tmp_g_score;
        pro_node->input = input;
        pro_node->parent = cur_node;
        pro_node->node_state = IN_OPEN_SET;
        pro_node->singul = singul;
        open_set_.push(pro_node);

        expanded_nodes_.insert(pro_id, pro_yaw_id, pro_node);
        use_node_num_ += 1;
        if (use_node_num_ == allocate_num_)
        {
          std::cout << "run out of memory." << std::endl;
          return NO_PATH;
        }
      }
      else if (pro_node->node_state == IN_OPEN_SET){
        if (tmp_g_score < pro_node->g_score){
          pro_node->index = pro_id;
          pro_node->state = pro_state;
          pro_node->yaw_idx = pro_yaw_id;
          pro_node->f_score = tmp_f_score;
          pro_node->g_score = real_g_score;
          pro_node->penalty_score = tmp_g_score;
          pro_node->input = input;
          pro_node->parent = cur_node;
          pro_node->singul = singul;
        }
      }
      else{
        std::cout << "error type in searching: " << pro_node->node_state << std::endl;
      }
    }
  }
  std::cout << "open set empty, no path." << std::endl;
  return NO_PATH;
}

bool KinoAstar::IfCollision_PosYaw(const Eigen::Vector2d pos, const double yaw){
// ROS_INFO("\033[43;37m IfCollision_PosYaw:  pos:%f %f %f yaw:%f Collision_interval_:%f \033[0m",pos.x(),pos.y(),stable_z_,yaw,Collision_interval_);  
  bool occ = false;
  Eigen::Vector2d point;
  uint8_t check_occ;
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  Eigen::Vector2d center(pos.x()+(length_/2-rear_suspension_)*cos_yaw, pos.y()+(length_/2-rear_suspension_)*sin_yaw);
  Eigen::Vector2d corner1(center.x() + 0.5*length_*cos_yaw + 0.5*width_*sin_yaw,
                          center.y() + 0.5*length_*sin_yaw - 0.5*width_*cos_yaw);
  Eigen::Vector2d corner2(center.x() + 0.5*length_*cos_yaw - 0.5*width_*sin_yaw,
                          center.y() + 0.5*length_*sin_yaw + 0.5*width_*cos_yaw);
  Eigen::Vector2d corner3(center.x() - 0.5*length_*cos_yaw - 0.5*width_*sin_yaw,
                          center.y() - 0.5*length_*sin_yaw + 0.5*width_*cos_yaw);
  Eigen::Vector2d corner4(center.x() - 0.5*length_*cos_yaw + 0.5*width_*sin_yaw,
                          center.y() - 0.5*length_*sin_yaw - 0.5*width_*cos_yaw);

// ROS_INFO("\033[41;37m x_upper:%f x_lower:%f y_upper:%f y_lower:%f z_upper:%f z_lower:%f \033[0m",x_upper,x_lower,y_upper,y_lower,z_upper,z_lower);
  
  check_occ = map_->CheckCollisionBycoord(corner1.x(),corner1.y(),height_/2);
  if(check_occ==1){
    return true;
  }
  check_occ = map_->CheckCollisionBycoord(corner2.x(),corner2.y(),height_/2);
  if(check_occ==1){
    return true;
  }
  check_occ = map_->CheckCollisionBycoord(corner3.x(),corner3.y(),height_/2);
  if(check_occ==1){
    return true;
  }
  check_occ = map_->CheckCollisionBycoord(corner4.x(),corner4.y(),height_/2);
  if(check_occ==1){
    return true;
  }

  double norm12 = (corner2-corner1).norm();
  double norm23 = (corner3-corner2).norm();
  double norm34 = (corner4-corner3).norm();
  double norm41 = (corner1-corner4).norm();
// ROS_INFO("\033[43;37m norm12:%f norm23:%f norm34:%f norm41:%f \033[0m",norm12,norm23,norm34,norm41);  
  for(double dl = Collision_interval_;dl < norm12;dl+=Collision_interval_){
    point = dl / norm12 * (corner2-corner1) + corner1;
    check_occ = map_->CheckCollisionBycoord(point.x(),point.y(),height_/2);
    if(check_occ==1){
      return true;
    }
  }
  for(double dl = Collision_interval_;dl < norm23;dl+=Collision_interval_){
    point = dl / norm23 * (corner3-corner2) + corner2;
    check_occ = map_->CheckCollisionBycoord(point.x(),point.y(),height_/2);
    if(check_occ==1){
      return true;
    }
  }
  for(double dl = Collision_interval_;dl < norm34;dl+=Collision_interval_){
    point = dl / norm34 * (corner4-corner3) + corner3;
    check_occ = map_->CheckCollisionBycoord(point.x(),point.y(),height_/2);
    if(check_occ==1){
      return true;
    }
  }
  for(double dl = Collision_interval_;dl < norm41;dl+=Collision_interval_){
    point = dl / norm41 * (corner1-corner4) + corner4;
    check_occ = map_->CheckCollisionBycoord(point.x(),point.y(),height_/2);
    if(check_occ==1){
      return true;
    }
  }

  return false;
}

int KinoAstar::yawToIndex(double yaw){
  yaw = normalize_angle(yaw);
  int idx = floor((yaw - yaw_origin_) * inv_yaw_resolution_);
  return idx;
}

double KinoAstar::normalize_angle(const double angle){
  double nor_angle = angle;
  nor_angle -=(angle >= M_PI) * 2 * M_PI;
  nor_angle -=(angle >= M_PI) * 2 * M_PI;
  return nor_angle;
}

// 初始速度大于0 小于0 几乎是0，从而给定初始方向
inline int KinoAstar::getSingularity(double vel){
  int singul = 0;
  if (fabs(vel) > non_siguav_){
    if (vel >= 0.0){singul = 1;} 
    else{singul = -1;}      
  }
  
  return singul;
}


inline double KinoAstar::getHeu(Eigen::VectorXd x1, Eigen::VectorXd x2){
  double dx = abs(x1(0) - x2(0));
  double dy = abs(x1(1) - x2(1));
  // return tie_breaker_* sqrt(dx * dx + dy * dy); //这是原文的
  // return sqrt(dx * dx + dy * dy) + fabs(x1(2)-x2(2))/10;
  return sqrt(dx * dx + dy * dy);
}

// state1是当前状态，state2是终点状态
bool KinoAstar::is_shot_sucess(Eigen::Vector3d state1,Eigen::Vector3d state2){
  std::vector<Eigen::Vector3d> path_list;
  double len,st;

  double ct1 = ros::Time::now().toSec();

  // st是以最快速度冲向目标点的时间
  st = computeShotTraj(state1,state2,path_list,len);

  double ct2 = ros::Time::now().toSec();
  std::cout<<"compute shot traj time: "<<(ct2-ct1)*1000.0<<" ms"<<std::endl;

  bool is_occ = false;
  // double t1 = ros::Time::now().toSec();
  // 检查全部采样的中间点是不是有碰撞
  for(unsigned int i = 0; i < path_list.size(); ++i){
    if(IfCollision_PosYaw(path_list[i].head(2),path_list[i][2]) == 1){
      return false;
    }
  }
  // double t2 = ros::Time::now().toSec();
  // std::cout<<"check collision time: "<<(t2-t1)*1000.0<<" ms"<<std::endl;
  // ros::Duration(10000.0).sleep();
  is_shot_succ_ = true;
  return true;
}

double KinoAstar::computeShotTraj(Eigen::Vector3d &state1, Eigen::Vector3d &state2,
                                  std::vector<Eigen::Vector3d> &path_list,
                                  double& len){
  namespace ob = ompl::base;
  namespace og = ompl::geometric;
  ob::ScopedState<> from(shotptr_), to(shotptr_), s(shotptr_);
  from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
  to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
  std::vector<double> reals;
  // 就是普通的计算距离  但是这里已经算完了？？
  len = shotptr_->distance(from(), to());
  double sum_T = len/max_vel_;    

  // 采样获得中间点
  for (double l = 0.0; l <=len; l += checkl_)
  {
    shotptr_->interpolate(from(), to(), l/len, s());
    reals = s.reals();
    path_list.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
  }

  return sum_T;
}


// to retrieve the path to the correct order 回退搜索路径
void KinoAstar::retrievePath(PathNodePtr end_node){
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

// // 根据输入获得之后的状态
// void KinoAstar::stateTransit(const Eigen::Vector3d &state0, const Eigen::Vector2d &ctrl_input, 
//                              Eigen::Vector3d &state1){
//   double psi = ctrl_input[0]; double s = ctrl_input[1]; 
//   if(psi!=0){
//     double k = wheel_base_/tan(psi);
//     state1[0] = state0[0] + k*(sin(state0[2]+s/k)-sin(state0[2]));
//     state1[1] = state0[1] - k*(cos(state0[2]+s/k)-cos(state0[2]));
//     state1[2] = state0[2] + s/k;
//   }
//   else{
//     state1[0] = state0[0] + s * cos(state0[2]);
//     state1[1] = state0[1] + s * sin(state0[2]);
//     state1[2] = state0[2]; 
//   }
// }

// 根据输入获得之后的状态
void KinoAstar::stateTransit(const Eigen::Vector3d &state0, const Eigen::Vector2d &ctrl_input, 
                             Eigen::Vector3d &state1){
  double sl = ctrl_input[0]; double sr = ctrl_input[1]; 
  double l = sl+sr;
  double theta = (sr-sl)/tread_;
  double ox,oy;
  if(fabs(theta)>1e-3){
    double r =  l / (2*theta);
    ox = 2*r*sin(theta)*cos(theta);
    oy = 2*r*sin(theta)*sin(theta);
  }
  else{
    ox = l;
    oy = 0;
  }
  state1[0] = state0[0] + ox*cos(state0[2])-oy*sin(state0[2]);
  state1[1] = state0[1] + oy*cos(state0[2])+ox*sin(state0[2]);
  state1[2] = state0[2] + theta;
}

void KinoAstar::NodeVis(const Eigen::Vector3d state){
  sensor_msgs::PointCloud2 globalMap_pcd;
  pcl::PointCloud<pcl::PointXYZ> cloudMap;
  pcl::PointXYZ pt;
  pt.x = state[0];
  pt.y = state[1];
  pt.z = 0.2;
  cloudMap.points.push_back(pt); 
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.stamp = ros::Time::now();
  globalMap_pcd.header.frame_id = "base";
  expandNodes_pub_.publish(globalMap_pcd);



  return;
}


void KinoAstar::getKinoNode(){
  // 采样点
  std::vector<Eigen::Vector3d> roughSampleList;//x,y,yaw
  // 获得起点速度和终点速度
  double startvel = fabs(start_state_[3]);
  double endvel = fabs(end_state_[3]);
  // 终点
  PathNodePtr node = path_nodes_.back();
// ROS_INFO("\033[40;37m getKinoNode  FINAL node:%f %f \033[0m",node->state.x(),node->state.y()); 
  std::vector<Eigen::Vector3d> traj_pts; // 储存均匀采样后的坐标点 x,y,t
  std::vector<double> thetas; // 储存均匀采样后的yaw
  Eigen::Vector4d x0, xt;
  Eigen::Vector2d ut, u0;
  // 从末尾的点开始  从后往前将前端的点及其用于碰撞检测的采样点都放进roughSampleList（为了让点更密集？）
  // roughSampleList里点的信息：x,y,theta
  while(node->parent != NULL){
    
    for (int k = check_num_; k >0; k--)
    {
      Eigen::Vector3d state;
      double vl = node->input[0] * double(k) / double(check_num_);
      double vr = node->input[1] * double(k) / double(check_num_);
      Eigen::Vector2d tmpctrl; tmpctrl << vl,vr;
      stateTransit(node->parent->state, tmpctrl, state);
      state[2] = normalize_angle(state[2]);
      roughSampleList.push_back(state);
    }
    node = node->parent;
  } 

  // 起点也放进去

  start_state_[2] = normalize_angle(start_state_[2]);
  roughSampleList.push_back(start_state_.head(3));
  reverse(roughSampleList.begin(),roughSampleList.end()); // 从起点开始到终点
ROS_INFO("\033[40;37m start_state_:%f %f \033[0m",roughSampleList.begin()->x(),roughSampleList.begin()->y()) ; 
ROS_INFO("\033[40;37m end_state_:%f %f \033[0m",roughSampleList[roughSampleList.size()-1].x(),roughSampleList[roughSampleList.size()-1].y()); 

  // 将oneshot的检查点全放进去
  if(is_shot_succ_){
    ompl::base::ScopedState<> from(shotptr_), to(shotptr_), s(shotptr_);
    Eigen::Vector3d state1,state2;
    state1 = roughSampleList.back();//前端终点
    state2 = end_state_.head(3);//实际终点
    from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
    to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
    double shotLength = shotptr_->distance(from(), to());//oneshot的距离
    std::vector<double> reals;
    // 采样获得中间点
    for(double l = checkl_; l < shotLength; l += checkl_){
      shotptr_->interpolate(from(), to(), l/shotLength, s());// 获得对应长度的中间点
      reals = s.reals();
      roughSampleList.push_back(Eigen::Vector3d(reals[0], reals[1], normalize_angle(reals[2])));
    }
    end_state_[2] = normalize_angle(end_state_[2]);
    roughSampleList.push_back(end_state_.head(3));
  }
  ROS_INFO("\033[40;37m end_state_:%f %f \033[0m",roughSampleList[roughSampleList.size()-1].x(),roughSampleList[roughSampleList.size()-1].y()); 
  //设定一个距离来截断初始轨迹  限制轨迹长度
  double tmp_len = 0;// 存储从起点开始到当前点的距离
  int truncate_idx = 0;
  for(truncate_idx = 0;truncate_idx <roughSampleList.size()-1;truncate_idx++){  
    tmp_len += (roughSampleList[truncate_idx+1].head(2)-roughSampleList[truncate_idx].head(2)).norm();
    if(tmp_len>max_kinoastar_dis_){
      break;
    }
  }
  roughSampleList.assign(roughSampleList.begin(),roughSampleList.begin()+truncate_idx+1);
  sample_trajs_ = roughSampleList;

  // 将整条轨迹变为多个部分
  std::vector<double> shot_lengthList;// 那些段的距离
  std::vector<double> shot_timeList;// 梯形速度算时间
  std::vector<int> shotindex;// 转折点在数组的坐标
  std::vector<int> shot_SList;// 存储每一段的方向
  double tmpl = 0;
  bool ifnewtraj = false;
  // 前进方向  通过比较两个点的向量与当前点的方向进行比较获得前进方向
  int lastS = (sample_trajs_[1]-sample_trajs_[0]).head(2).dot(Eigen::Vector2d(cos(sample_trajs_[0][2]),sin(sample_trajs_[0][2])))>=0?1:-1;
  shotindex.push_back(0);
ROS_INFO("\033[40;37m gui ji fen duan!! \033[0m"); 
  for(int i = 0; i<sample_trajs_.size()-1; i++){
    Eigen::Vector3d state1 = sample_trajs_[i];
    Eigen::Vector3d state2 = sample_trajs_[i+1];
    int curS = (state2-state1).head(2).dot(Eigen::Vector2d(cos(state1[2]),sin(state1[2]))) >=0 ? 1:-1;// 当前前进方向
    // 前进方向不变则往前继续计算长度，前进方向变则保存这个转折点
    if(curS*lastS >= 0){
      tmpl += (state2-state1).head(2).norm();
    }
    else{ 
      // 储存转折点位置
      shotindex.push_back(i);
      // 储存转折前方向
      shot_SList.push_back(lastS);
      // 储存转折前总距离
      shot_lengthList.push_back(tmpl);
      // 储存转折前时间
      shot_timeList.push_back(evaluateDuration(tmpl,non_siguav_,non_siguav_));
      tmpl = (state2-state1).head(2).norm();//重新开始计算距离
    }       
    lastS = curS;
  }
  // 将结尾的路径放进去
  shot_SList.push_back(lastS);
  shot_lengthList.push_back(tmpl);
  shot_timeList.push_back(evaluateDuration(tmpl,non_siguav_,non_siguav_));
  shotindex.push_back(sample_trajs_.size()-1);
  // 改时间 考虑初末的速度
  if(shot_timeList.size()>=2){
    shot_timeList[0] = evaluateDuration(shot_lengthList[0],startvel,non_siguav_);
    shot_timeList[shot_timeList.size()-1] = evaluateDuration(shot_lengthList.back(),non_siguav_,endvel);
  }
  else{
    shot_timeList[0] = evaluateDuration(shot_lengthList[0],startvel,endvel);
  }
  // 
  ROS_INFO("\033[40;37m getting traj_pts!! \033[0m"); 
  for(int i=0;i<shot_lengthList.size();i++){

    double initv = non_siguav_,finv = non_siguav_;
    Eigen::Vector2d Initctrlinput,Finctrlinput;
    Initctrlinput<<0,0;Finctrlinput<<0,0;
    // 读取初始速度和末速度
    if(i==0) {initv  = startvel; Initctrlinput = start_ctrl_;}
    if(i==shot_lengthList.size() - 1) finv = endvel;

    double locallength = shot_lengthList[i];//第i段距离
    int sig = shot_SList[i];//第i段方向
    // 将第i段方向的所有点择出来
    std::vector<Eigen::Vector3d> localTraj;
    localTraj.assign(sample_trajs_.begin()+shotindex[i],sample_trajs_.begin()+shotindex[i+1]+1);
    traj_pts.clear();
    thetas.clear();        
    double samplet;
    double tmparc = 0;
    int index = 0;
    // 如果这段轨迹太短 则至少分为两份
    double sampletime = sampletime_;
    if(shot_timeList[i]<=sampletime_){
      sampletime = shot_timeList[i] / 2.0;
    }
    if(shot_timeList[i]<=1e-3){
      continue;
    }
    //按照采样时间对轨迹进行均匀采样
    for(samplet = sampletime; samplet<shot_timeList[i]; samplet+=sampletime){
      ROS_INFO("\033[40;36m getting traj_pts!!  samplet: %f   sampletime: %f\033[0m",samplet,sampletime); 
      // 在采样时间的采样
      double arc = evaluateLength(samplet,locallength,shot_timeList[i], initv, finv);
      // 找到最接近这个距离的两个点并且差值获得该点
      for(int k = index; k<localTraj.size()-1 ;k++)
      {
        // 找到距离刚好是arc的点
        tmparc+= (localTraj[k+1]-localTraj[k]).head(2).norm();
        if(tmparc>=arc){
          index = k;
          double l1 = tmparc-arc;
          double l = (localTraj[k+1]-localTraj[k]).head(2).norm();
          double l2 = l-l1;//l2
          double px = (l1/l*localTraj[k]+l2/l*localTraj[k+1])[0];
          double py = (l1/l*localTraj[k]+l2/l*localTraj[k+1])[1];
          double yaw= (l1/l*localTraj[k]+l2/l*localTraj[k+1])[2];
          if(fabs(localTraj[k+1][2]-localTraj[k][2])>=M_PI){   
            double normalize_yaw;
            if(localTraj[k+1][2]<=0){
              normalize_yaw = l1/l*localTraj[k][2]+l2/l*(localTraj[k+1][2]+2*M_PI);
            }
            else if(localTraj[k][2]<=0){
              normalize_yaw = l1/l*(localTraj[k][2]+2*M_PI)+l2/l*localTraj[k+1][2];
            }
            yaw = normalize_yaw;
          }
          traj_pts.push_back(Eigen::Vector3d(px,py,sampletime));
          thetas.push_back(yaw);
          tmparc -=(localTraj[k+1]-localTraj[k]).head(2).norm();
          break;
        }
      }
    }
    traj_pts.push_back(Eigen::Vector3d(localTraj.back()[0],localTraj.back()[1],shot_timeList[i]-(samplet-sampletime)));
    thetas.push_back(localTraj.back()[2]);
    
    // flat_trajs_
    Eigen::MatrixXd startS;
    Eigen::MatrixXd endS;
    getFlatState(Eigen::Vector4d(localTraj.front()[0],localTraj.front()[1],localTraj.front()[2],initv),Initctrlinput,sig,startS);
    getFlatState(Eigen::Vector4d(localTraj.back()[0],localTraj.back()[1],localTraj.back()[2],finv),Finctrlinput,sig,endS);
    
    FlatTrajData flat_traj;
    
    flat_traj.traj_pts = traj_pts;
    flat_traj.thetas = thetas;
    flat_traj.start_state = startS;
    flat_traj.final_state = endS;
    flat_traj.singul = sig;
    flat_trajs_.push_back(flat_traj);
  }
ROS_INFO("\033[40;37m get kino node over! \033[0m"); 
}

// 利用梯形速度获得时间
double KinoAstar::evaluateDuration(const double &length, const double &startV, const double &endV){
  double critical_len; 
  if(startV>max_vel_||endV>max_vel_){
    ROS_ERROR("kinoAstar:evaluateDuration:start or end vel is larger that the limit!");
  }
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(max_vel_,2);
  critical_len = (maxv2-startv2)/(2*max_acc_)+(maxv2-endv2)/(2*max_acc_);
  if(length>=critical_len){
    return (max_vel_-startV)/max_acc_+(max_vel_-endV)/max_acc_+(length-critical_len)/max_vel_;
  }
  else{
    double tmpv = sqrt(0.5*(startv2+endv2+2*max_acc_*length));
    return (tmpv-startV)/max_acc_ + (tmpv-endV)/max_acc_;
  }
}

// 利用梯形速度获得curt时间戳处的距离
double KinoAstar::evaluateLength(const double &curt, const double &locallength, const double &localtime, const double &startV, const double &endV){
  double critical_len; 
  if(startV>max_vel_||endV>max_vel_){
    ROS_ERROR("kinoAstar:evaluateLength:start or end vel is larger that the limit!");
  }
  double startv2 = pow(startV,2);
  double endv2 = pow(endV,2);
  double maxv2 = pow(max_vel_,2);
  critical_len = (maxv2-startv2)/(2*max_acc_)+(maxv2-endv2)/(2*max_acc_);
  // 从梯形速度获得时间
  if(locallength>=critical_len){// 如果locallength距离比critical_len大，则为加速到到最大速度再减速
    double t1 = (max_vel_-startV)/max_acc_;
    double t2 = t1+(locallength-critical_len)/max_vel_;
    if(curt<=t1){
      return startV*curt + 0.5*max_acc_*pow(curt,2);
    }
    else if(curt<=t2){
      return startV*t1 + 0.5*max_acc_*pow(t1,2)+(curt-t1)*max_vel_;
    }
    else{
      return startV*t1 + 0.5*max_acc_*pow(t1,2) + (t2-t1)*max_vel_ + max_vel_*(curt-t2)-0.5*max_acc_*pow(curt-t2,2);
    }
  }
  else{//如果locallength距离比critical_len小，则未加速到到最大速度就减速
    double tmpv = sqrt(0.5*(startv2+endv2+2*max_acc_*locallength));
    double tmpt = (tmpv-startV)/max_acc_;
    if(curt<=tmpt){
      return startV*curt+0.5*max_acc_*pow(curt,2);
    }
    else{
      return startV*tmpt+0.5*max_acc_*pow(tmpt,2) + tmpv*(curt-tmpt)-0.5*max_acc_*pow(curt-tmpt,2);
    }
  }
}

// state:x y yaw v   flat_state: p v a
void KinoAstar::getFlatState(const Eigen::Vector4d &state, const Eigen::Vector2d &control_input,const int &singul, 
                             Eigen::MatrixXd &flat_state){

    flat_state.resize(2, 3);

    double angle = state(2);
    double vel   = state(3); // vel > 0 

    Eigen::Matrix2d init_R;
    init_R << cos(angle),  -sin(angle),
              sin(angle),   cos(angle);

    if (abs(vel) <= non_siguav_){
      vel = singul * non_siguav_;
    }
    else{
      vel = singul * vel;
    }
    //// 这里把前进距离当做加速度奇奇怪怪的
    flat_state << state.head(2), init_R*Eigen::Vector2d(vel, 0.0), 
                  init_R*Eigen::Vector2d(control_input(1), std::tan(control_input(0)) / wheel_base_ * std::pow(vel, 2));
}


bool KinoAstar::IfCollision_Toward(const Eigen::Vector2d &pos, const double &yaw){
  bool occ = false;
  Eigen::Vector2d point;
  uint8_t check_occ;
  double cos_yaw = cos(yaw);
  double sin_yaw = sin(yaw);
  Eigen::Vector2d center(pos.x()+traj_toword_occ_scope_*cos_yaw, pos.y()+traj_toword_occ_scope_*sin_yaw);
  Eigen::Vector2d toward(center.x()+traj_toword_occ_scope_*cos_yaw, pos.y()+traj_toword_occ_scope_*sin_yaw);
  double dis = (center-toward).norm();
  for(double dl = Collision_interval_;dl < dis;dl+=Collision_interval_){
    point = dl / dis * (toward-center) + center;
    check_occ = map_->CheckCollisionBycoord(point.x(),point.y(),height_/2);
    if(check_occ==1){
      return true;
    }
  }

  return false;
}