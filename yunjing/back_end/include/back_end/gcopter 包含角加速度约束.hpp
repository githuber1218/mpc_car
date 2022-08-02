#ifndef _GCOPTER_HPP_
#define _GCOPTER_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

#include "model_gazebo/world2oct.h"
#include "front_end/traj_representation.h"
#include "front_end/kino_astar.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "gcopter/trajectory.hpp"
#include "gcopter/minco.hpp"
// #include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
// #include "gcopter/flatness.hpp"
// #include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"


// TODO:
// 1.缺一个reset函数
// 2. 给minco的setup还没写完呢

struct Config
{
    // 车辆参数
    double max_vel_;
    double max_acc_;
    double max_cur_;
    double max_domega_;
    double tread_;
    double front_suspension_;
    double rear_suspension_;
    double wheel_base_;
    double base_height_;
    double length_;
    double width_;
    double height_;

    // 走廊参数
    int denseResolution_;
    int sparseResolution_;
    double timeResolution_;




    Config(const ros::NodeHandle &nh_)
    {
      
      nh_.param<double>(ros::this_node::getName()+ "/height",height_,0.5);
      nh_.param<double>(ros::this_node::getName()+ "/length",length_,0.4);
      nh_.param<double>(ros::this_node::getName()+ "/width",width_,0.4);
      nh_.param<double>(ros::this_node::getName()+ "/max_vel",max_vel_,5);
      nh_.param<double>(ros::this_node::getName()+ "/max_acc",max_acc_,5);
      nh_.param<double>(ros::this_node::getName()+ "/max_cur",max_cur_,0.5);
      nh_.param<double>(ros::this_node::getName()+ "/max_domega",max_domega_,50);
      nh_.param<double>(ros::this_node::getName()+ "/tread",tread_,0.3);
      nh_.param<double>(ros::this_node::getName()+ "/front_suspension",front_suspension_,0.05);
      nh_.param<double>(ros::this_node::getName()+ "/rear_suspension",rear_suspension_,0.05);
      nh_.param<double>(ros::this_node::getName()+ "/wheel_base",wheel_base_,0.8);

      nh_.param<int>(ros::this_node::getName()+ "/denseResolution",denseResolution_,20);
      nh_.param<int>(ros::this_node::getName()+ "/sparseResolution",sparseResolution_,8);
      nh_.param<double>(ros::this_node::getName() + "/timeResolution",timeResolution_,1);

      // ROS_INFO("max_cur_:%d " ,max_cur_);
    }
};

class Gcopter
{
private:
  Config config_;
  ros::NodeHandle nh_;
  std::shared_ptr<World2oct> map_;
  std::shared_ptr<KinoAstar> kinoastar_;
  



  ros::Publisher meshPub;
  ros::Publisher edgePub;
  ros::Publisher box2DPub;
  ros::Publisher point_pub;
  ros::Publisher leftedgePub;
  ros::Publisher rightedgePub;

  // 优化参数
  double rho;
  double smoothEps;// smoothL1的范围
  // double integralRes;// 积分分段数
  Eigen::VectorXd magnitudeBd;// v_weight  a_weight  cur_weight  omega_weight  colli_weight
  Eigen::VectorXd penaltyWt;
  double safeDis;

  // std::vector<std::vector<Eigen::MatrixX4d>> hPolyses;
  // std::vector<std::vector<double>> hPolyTimeses;
  // std::vector<std::vector<Eigen::Vector2d>> Innerpointses;
  // std::vector<int> hPolynums;
  // std::vector<int> singuls;

     
  int piece_singul_num_;
  Eigen::VectorXd pieceTimes;

  std::vector<Eigen::MatrixXd> Innerpointses;
  Eigen::VectorXi singuls;
  // 大段 小分段 某段的走廊
  std::vector<std::vector<Eigen::MatrixXd>> hPolyses;
  std::vector<std::vector<Eigen::MatrixXd>> leftManiHpolyses;
  std::vector<std::vector<Eigen::MatrixXd>> rightManiHpolyses;
  std::vector<Eigen::MatrixXd> iniStates;
  std::vector<Eigen::MatrixXd> finStates;
  Eigen::VectorXi eachTrajNums;

  std::vector<Eigen::MatrixXd> finalInnerpointses;

  // 自由度
  double Freedom_;

  double range;

  // // int piece_num_;
  // std::vector<Eigen::MatrixXd> headPVA_;
  // std::vector<Eigen::MatrixXd> tailPVA_;

  // 需要删除内存!!!!!!!!!!!!!!!!!!!!!!!!!TODO
  std::vector<minco::MINCO_S3NU> mincos;

  //statelists -> statelist ->state
  std::vector<std::vector<Eigen::Vector3d>> statelists;

  // 优化过程参数
  int iter_num_;
  // 储存梯度  对q、加上c的T、c、不加c的T
  Eigen::MatrixXd gradByPoints;//joint_piece_*(piece_num_-1)
  Eigen::VectorXd gradByTimes;
  Eigen::MatrixXd partialGradByCoeffs;//(2s*piece_num_) * joint_piece_
  Eigen::VectorXd partialGradByTimes;
  double gradOneTime;

  std::vector<Eigen::Vector2d> check_point;
  std::vector<Eigen::Vector3d> leftMani_check_point;
  std::vector<Eigen::Vector3d> rightMani_check_point;

  bool ifprint = false;

public:

  // 结果
  Trajectory<5, 2> final_traj;
  std::vector<Trajectory<5, 2>> final_trajes;
  // Eigen::VectorXd final_time;
  Eigen::VectorXi final_singuls;
  double offsetx;
  // inline bool setup(std::vector<FlatTrajData> &flat_trajs_){
  //   nh_.param<double>(ros::this_node::getName()+ "/TimeWeight", rho, 1);
  //   nh_.param<double>(ros::this_node::getName()+ "/smoothingFactor", smoothEps, 0.01);
  //   nh_.param<double>(ros::this_node::getName()+ "/integralResolution", integralRes, 12);

  //   std::vector<double> magBd;
  //   nh_.getParam(ros::this_node::getName()+ "/magnitudeBounds",magBd);
  //   magnitudeBd.resize(magBd.size());
  //   for(int i=0; i<magBd.size(); i++){
  //     magnitudeBd[i] = magBd[i];
  //   }

  //   std::vector<double> penaWt;
  //   nh_.getParam(ros::this_node::getName()+ "/penaltyWeights",penaWt);
  //   penaltyWt.resize(penaWt.size());
  //   for(int i=0; i<penaWt.size(); i++){
  //     penaltyWt[i] = penaWt[i];
  //   }

  //   piece_num_ = flat_trajs_.size();
  //   headPVA_.clear();
  //   tailPVA_.clear();
  //   singul_.clear();
  //   mincos.clear();
  //   for(int i=0;i<piece_num_;i++){
  //     Eigen::MatrixXd headPVA;
  //     headPVA.resize(Freedom_,3);
  //     Eigen::MatrixXd tailPVA;
  //     tailPVA.resize(Freedom_,3);
  //     headPVA = flat_trajs_[i].start_state;
  //     tailPVA = flat_trajs_[i].final_state;
  //     headPVA_.push_back(headPVA);
  //     tailPVA_.push_back(tailPVA);
  //     singul_.push_back(flat_trajs_[i].singul);
  //     hPolynum.push_back(hPolyses[i].size());
    
  //     minco::MINCO_S3NU minco;
  //     // minco.setConditions(headPVA,tailPVA)
  //     // 没写完！！！！！！！！！！！！！！！！！！！！！！！！！！！
  //   }



  // }


  Gcopter(const Config &conf, ros::NodeHandle &nh, std::shared_ptr<World2oct> map, std::shared_ptr<KinoAstar> kinoastar):config_(conf){
    Freedom_ = 2;
    nh_ = nh;
    map_ = map;
    kinoastar_ = kinoastar;
    meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
leftedgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/leftedge", 1000);
rightedgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/rightedge", 1000);

    
    box2DPub = nh.advertise<visualization_msgs::Marker>("/visualizer/box2D", 1000);
    point_pub = nh_.advertise<visualization_msgs::MarkerArray>("/visualizer/innerpoint",100);



    // 读参数
    nh_.param<double>(ros::this_node::getName()+ "/TimeWeight", rho, 1);
    nh_.param<double>(ros::this_node::getName()+ "/smoothingFactor", smoothEps, 0.01);
    nh_.param<double>(ros::this_node::getName()+ "/safeDis", safeDis, 0);
    nh_.param<double>(ros::this_node::getName()+ "/range", range, 1);
    // nh_.param<double>(ros::this_node::getName()+ "/integralResolution", integralRes, 12);

    std::vector<double> magBd;
    nh_.getParam(ros::this_node::getName()+ "/magnitudeBounds",magBd);
    magnitudeBd.resize(magBd.size());
    for(int i=0; i<magBd.size(); i++){
      magnitudeBd[i] = magBd[i];
    }

    std::vector<double> penaWt;
    nh_.getParam(ros::this_node::getName()+ "/penaltyWeights",penaWt);
    penaltyWt.resize(penaWt.size());
    for(int i=0; i<penaWt.size(); i++){
      penaltyWt[i] = penaWt[i];
    }

    check_point.clear();
    check_point.emplace_back(config_.length_-config_.rear_suspension_,config_.width_/2);
    check_point.emplace_back(config_.length_-config_.rear_suspension_,-config_.width_/2);
    check_point.emplace_back(-config_.rear_suspension_,-config_.width_/2);
    check_point.emplace_back(-config_.rear_suspension_,config_.width_/2);

    leftMani_check_point.clear();
    leftMani_check_point.emplace_back(0.025 + config_.length_/2-config_.rear_suspension_,0.1,0.3);
    leftMani_check_point.emplace_back(0.025 + config_.length_/2-config_.rear_suspension_,0.15,0.3);
    leftMani_check_point.emplace_back(-0.025 + config_.length_/2-config_.rear_suspension_,0.15,0.3);
    leftMani_check_point.emplace_back(-0.025 + config_.length_/2-config_.rear_suspension_,0.1,0.3);
    leftMani_check_point.emplace_back(0.025 + config_.length_/2-config_.rear_suspension_,0.1,0.45);
    leftMani_check_point.emplace_back(0.025 + config_.length_/2-config_.rear_suspension_,0.15,0.45);
    leftMani_check_point.emplace_back(-0.025 + config_.length_/2-config_.rear_suspension_,0.15,0.45);
    leftMani_check_point.emplace_back(-0.025 + config_.length_/2-config_.rear_suspension_,0.1,0.45);

    rightMani_check_point.clear();
    rightMani_check_point.emplace_back(0.025 + config_.length_/2-config_.rear_suspension_,-0.1,0.3);
    rightMani_check_point.emplace_back(0.025 + config_.length_/2-config_.rear_suspension_,-0.15,0.3);
    rightMani_check_point.emplace_back(-0.025 + config_.length_/2-config_.rear_suspension_,-0.15,0.3);
    rightMani_check_point.emplace_back(-0.025 + config_.length_/2-config_.rear_suspension_,-0.1,0.3);
    rightMani_check_point.emplace_back(0.025 + config_.length_/2-config_.rear_suspension_,-0.1,0.45);
    rightMani_check_point.emplace_back(0.025 + config_.length_/2-config_.rear_suspension_,-0.15,0.45);
    rightMani_check_point.emplace_back(-0.025 + config_.length_/2-config_.rear_suspension_,-0.15,0.45);
    rightMani_check_point.emplace_back(-0.025 + config_.length_/2-config_.rear_suspension_,-0.1,0.45);

    offsetx = config_.length_/2 - config_.rear_suspension_;
  }



  inline void minco_plan(){
    if(!(kinoastar_->has_path_)){
      ROS_ERROR("there is no kinoastar path!!!!!!!!!!!!");
      return;
    }

    get_hPoly();
    vis_2Dbox();
    // vis_3Dbox();
    vis_right3Dbox();
    vis_left3Dbox();
    vis_innerPoint();
    // get_hPolys();

    // Innerpointses和轨迹数量测试是没发现问题的
    for(int i=0;i<eachTrajNums.size();i++){
      std::cout<<"Innerpointses.size():"<<Innerpointses[i].cols()<<std::endl;
      std::cout<<"eachTrajNums:"<<eachTrajNums[i]<<std::endl<<std::endl;
    }
    optimizer();
    vis_finalinnerPoint();
  }

  inline void get_hPoly(){
    
    hPolyses.clear();

    leftManiHpolyses.clear();
    rightManiHpolyses.clear();
    
    Innerpointses.clear();
    iniStates.clear();
    finStates.clear();
    finalInnerpointses.clear();

    // 存储再次均分后每一段的时间 是一个固定值向量
    Eigen::VectorXd ego_piece_dur_vec;

    double basetime = 0.0;

    int traj_num = kinoastar_->flat_trajs_.size();
    piece_singul_num_ = traj_num;
    pieceTimes.resize(traj_num);
    singuls.resize(traj_num);
    eachTrajNums.resize(traj_num);
    // 储存每一段的所有中间点   2 * piece_nums-1
    Eigen::MatrixXd ego_innerPs;


    for(int i=0; i<traj_num; i++){
      // 传过来的第i段轨迹
      FlatTrajData kino_traj = kinoastar_->flat_trajs_.at(i);
      singuls[i] = kino_traj.singul;
      // 第i段轨迹的轨迹点
      std::vector<Eigen::Vector3d> pts = kino_traj.traj_pts;
      int piece_nums;
      // 第i段轨迹的时间（从前端传过来的）
      double initTotalduration = 0.0;
      for(const auto pt : pts){
        initTotalduration += pt[2];
      }
      // 重新分小段  四舍五入按照进行分段，但是至少分成两段
      piece_nums = std::max(int(initTotalduration / config_.timeResolution_ + 0.5),3);
      // 再均匀分小段
      double timePerPiece = initTotalduration / piece_nums; 
      ego_piece_dur_vec.resize(piece_nums);
      ego_piece_dur_vec.setConstant(timePerPiece);
      // 存下每一大段的总时间
      pieceTimes[i] = initTotalduration;
      ego_innerPs.resize(2, piece_nums-1);
      // 大段均匀分成小段再均分后的state
      std::vector<Eigen::Vector3d> statelist;
      double res_time = 0;
      // 循环小段
      for(int i = 0; i < piece_nums; i++ ){
        int resolution;
        // 遍历小段，在初始和末尾采样密集：20  中间采样稀疏：8
        if(i==0||i==piece_nums-1){
          resolution = config_.denseResolution_;
        }
        else{
          resolution = config_.sparseResolution_;
        }
        // 得到均匀的小段再均匀后的时间节点所对应的位置存到statelist，小段的末点放进ego_innerPs
        for(int k = 0; k <= resolution; k++){
          // 采样后第k小段的时间 basetime是总时间 res_time是循环小段的计时
          double t = basetime+res_time + 1.0*k/resolution*ego_piece_dur_vec[i];
          // 通过插值获得在时间t的状态坐标(x,y,yaw)
          Eigen::Vector3d pos = kinoastar_->evaluatePos(t);
          statelist.push_back(pos);
          if(k==resolution && i!=piece_nums-1){
            ego_innerPs.col(i) = pos.head(2); 
          }
        } 
        res_time += ego_piece_dur_vec[i];
      }
      // 将statelist里面的状态获得走廊放到hPolys_里面
      std::vector<Eigen::MatrixXd> hPolys;
      getRectangleConst(statelist, hPolys);
      
      std::vector<Eigen::MatrixXd> leftHPolys;
      std::vector<Eigen::MatrixXd> rightHPolys;
      getManiCorridor(statelist, leftHPolys, rightHPolys);
      leftManiHpolyses.push_back(leftHPolys);
      rightManiHpolyses.push_back(rightHPolys);

      hPolyses.push_back(hPolys);
      Innerpointses.push_back(ego_innerPs);
      iniStates.push_back(kino_traj.start_state);
      finStates.push_back(kino_traj.final_state);
      eachTrajNums[i] = piece_nums;
      basetime += initTotalduration;

      std::cout<<"ego_innerPs: "<<std::endl<<ego_innerPs.transpose()<<std::endl<<std::endl;
      std::cout<<"statelist:"<<std::endl;
      for(int debugi=0;debugi<statelist.size();debugi++){
        std::cout<<"debugi: "<<debugi<<"   "<<statelist[debugi].transpose()<<std::endl;
      }
      std::cout<<std::endl;
    }
    ROS_INFO("hPolyses.size(),Innerpointses.size(),iniStates.size(),finStates.size() %d %d %d %d",hPolyses.size(),Innerpointses.size(),iniStates.size(),finStates.size());
  }


  inline void optimizer(){
    ROS_INFO("start optimizer!");
    if(Innerpointses.size()!=piece_singul_num_ || singuls.size()!=piece_singul_num_ ||
       hPolyses.size()!=piece_singul_num_|| iniStates.size()!=piece_singul_num_ ||
       finStates.size()!=piece_singul_num_ || eachTrajNums.size()!=piece_singul_num_ ||
       pieceTimes.size()!=piece_singul_num_){
      ROS_ERROR("[optimizer ERROR]: Innerpointses.size()!=piece_singul_num_ || singuls.size()!=piece_singul_num_ || hPolyses.size()!=piece_singul_num_|| iniStates.size()!=piece_singul_num_ || finStates.size()!=piece_singul_num_ || eachTrajNums.size()!=piece_singul_num_");
      return;
    }

    int variable_num_ = 0;
    mincos.clear();
    mincos.resize(piece_singul_num_);
    ROS_INFO("minco::MINCO_S3NU Minco;");
    minco::MINCO_S3NU Minco;
    for(int i=0; i<piece_singul_num_; i++){
      if(Innerpointses[i].cols()==0){
        ROS_ERROR("[optimizer ERROR] no Innerpoint!");
        return;
      }
      int piece_num = eachTrajNums[i];
      if(hPolyses[i].size()!=(piece_num - 2) * (config_.sparseResolution_ + 1) + 2 * (config_.denseResolution_  + 1)){
        std::cout<<"cfgHs size: "<<hPolyses[i].size()<<std::endl;
        ROS_ERROR("cfgHs size error!");
        return;
      }

      if(iniStates[i].col(1).norm()>=config_.max_vel_){
        iniStates[i].col(1) = iniStates[i].col(1).normalized()*(config_.max_vel_-1.0e-2);
      }
      if(iniStates[i].col(2).norm()>=config_.max_acc_){
        iniStates[i].col(2) = iniStates[i].col(2).normalized()*(config_.max_acc_-1.0e-2);
      }
      if(finStates[i].col(1).norm()>=config_.max_vel_){
        finStates[i].col(1) = finStates[i].col(1).normalized()*(config_.max_vel_-1.0e-2);
      }
      if(finStates[i].col(2).norm()>=config_.max_acc_){
        finStates[i].col(2) = finStates[i].col(2).normalized()*(config_.max_acc_-1.0e-2);
      }
      
      // variable_num_ += 2 * (piece_num - 1) + 1;
      variable_num_ += 2 * piece_num - 1;

      ROS_INFO("setConditions");
      std::cout<<"iniStates[i]:  "<<iniStates[i]<<std::endl;
      std::cout<<"finStates[i]:  "<<finStates[i]<<std::endl;
      std::cout<<"piece_num:  "<<piece_num<<std::endl;
      std::cout<<"Freedom_:  "<<Freedom_<<std::endl;
      // Minco.setConditions(iniStates[i],finStates[i],piece_num,Freedom_);
      // ROS_INFO("push_back");
      // mincos.push_back(Minco);
      mincos[i].setConditions(iniStates[i],finStates[i],piece_num,Freedom_);
    }
    ROS_INFO("ini mincos over!");
    Eigen::VectorXd x;
    x.resize(variable_num_);
    int offset = 0;
    for(int i = 0; i<piece_singul_num_; i++){
      memcpy(x.data()+offset,Innerpointses[i].data(), Innerpointses[i].size() * sizeof(x[0]));
      offset += Innerpointses[i].size();
    }
    Eigen::Map<Eigen::VectorXd> Vt(x.data()+offset, pieceTimes.size());
    RealT2VirtualT(1.5*pieceTimes,Vt);

    

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 128;//128
    lbfgs_params.past = 20; //3 
    lbfgs_params.g_epsilon = 1.0e-16;
    // lbfgs_params.g_epsilon = 0.1;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.delta = 1.0e-6;
    lbfgs_params.max_iterations = 1000;

    iter_num_ = 0;
    double cost;
    ROS_INFO("go in lbfgs_optimize!");
    int result = lbfgs::lbfgs_optimize(x,
                                       cost,
                                       Gcopter::costFunctionCallback,
                                       NULL,
                                       NULL,
                                       this,
                                       lbfgs_params);
    std::cout<<"-------------------------------------------final---------------------------------------"<<std::endl;
    Eigen::VectorXd g;
    g.resize(x.size());
    ifprint = true;
    costFunctionCallback(this,x,g);
    
    offset = 0;
    // final_time.resize(piece_singul_num_);
    final_trajes.clear();
    std::cout<<"T:    ";
    for(int i=0; i<piece_singul_num_; i++){
      Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, eachTrajNums[i] - 1);
      offset += 2 * (eachTrajNums[i] - 1);
      double t = x[x.size()-piece_singul_num_+i];
      double T = t > 0
                    ? (t*(t/2+1)+1)
                    : (2/(t*(t-2)+2));
      // final_time[i] = T/eachTrajNums[i];
      mincos[i].setParameters(P,T/eachTrajNums[i]);
      mincos[i].getTrajectory(final_traj);
      final_trajes.push_back(final_traj);
      finalInnerpointses.emplace_back(P);
      std::cout<<" "<<T;
    }
    final_singuls = singuls;
    std::cout<<std::endl;
    std::cout<<"g: "<<g.transpose()<<std::endl;
    std::cout<<"eachTrajNums: "<<eachTrajNums.transpose()<<std::endl;
    ROS_INFO("\n\n optimizer finish! result:%d   finalcost:%f   iter_num_:%d\n\n ",result,cost,iter_num_);
    
  }


  void getRectangleConst(const std::vector<Eigen::Vector3d> &statelist, std::vector<Eigen::MatrixXd> &hPolys){
    // 最大搜索范围
    // double range = 0.5;
    
    int state_num = statelist.size();
    hPolys.clear();
    double resolution = map_->grid_interval_;
    for(int i=0; i<state_num; i++){
      Eigen::Matrix4d hPoly;
      double step = resolution;
      bool x_pos=true,y_pos=true,x_neg=true,y_neg=true;
      Eigen::Vector2d sourcePt = statelist[i].head(2);
      Eigen::Vector4d extend_length(config_.length_-config_.rear_suspension_,
                                    config_.width_/2,
                                    -config_.rear_suspension_,
                                    -config_.width_/2);
      double yaw = statelist[i][2];
      bool test;
      Eigen::Matrix2d egoR;
      egoR<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
      // test = kinoastar_->IfCollision_PosYaw(sourcePt,yaw);
      // if(test){
      //   ROS_ERROR("error! traj_pts collision!!!");
      //   return;
      // }
      // Eigen::Vector4d expandLength;
      // expandLength << 0.0, 0.0, 0.0, 0.0;
      Eigen::Vector2d Corpoint1,Corpoint2,Corpoint3,Corpoint4;
      while(x_pos || y_pos || x_neg || y_neg){
        if(x_pos){
          Corpoint1 = sourcePt + egoR*Eigen::Vector2d(extend_length[0],extend_length[1]);
          Corpoint2 = sourcePt + egoR*Eigen::Vector2d(extend_length[0]+resolution,extend_length[1]);
          Corpoint3 = sourcePt + egoR*Eigen::Vector2d(extend_length[0]+resolution,extend_length[3]);
          Corpoint4 = sourcePt + egoR*Eigen::Vector2d(extend_length[0],extend_length[3]);
          if(IfCollision_LineWithHeight(Corpoint1,Corpoint2) || IfCollision_LineWithHeight(Corpoint2,Corpoint3) || IfCollision_LineWithHeight(Corpoint3,Corpoint4) || extend_length[0]+resolution>range){
            x_pos = false;
          }
          else{
            extend_length[0]+=resolution;
          }
        }
        if(y_pos){
          Corpoint1 = sourcePt + egoR*Eigen::Vector2d(extend_length[2],extend_length[1]);
          Corpoint2 = sourcePt + egoR*Eigen::Vector2d(extend_length[2],extend_length[1]+resolution);
          Corpoint3 = sourcePt + egoR*Eigen::Vector2d(extend_length[0],extend_length[1]+resolution);
          Corpoint4 = sourcePt + egoR*Eigen::Vector2d(extend_length[0],extend_length[1]);
          if(IfCollision_LineWithHeight(Corpoint1,Corpoint2) || IfCollision_LineWithHeight(Corpoint2,Corpoint3) || IfCollision_LineWithHeight(Corpoint3,Corpoint4) || extend_length[1]+resolution>range){
            y_pos = false;
          }
          else{
            extend_length[1]+=resolution;
          }
        }
        if(x_neg){
          Corpoint1 = sourcePt + egoR*Eigen::Vector2d(extend_length[2],extend_length[1]);
          Corpoint2 = sourcePt + egoR*Eigen::Vector2d(extend_length[2]-resolution,extend_length[1]);
          Corpoint3 = sourcePt + egoR*Eigen::Vector2d(extend_length[2]-resolution,extend_length[3]);
          Corpoint4 = sourcePt + egoR*Eigen::Vector2d(extend_length[2],extend_length[3]);
          if(IfCollision_LineWithHeight(Corpoint1,Corpoint2) || IfCollision_LineWithHeight(Corpoint2,Corpoint3) || IfCollision_LineWithHeight(Corpoint3,Corpoint4)|| -(extend_length[2]-resolution)>range){
            x_neg = false;
          }
          else{
            extend_length[2]-=resolution;
          }
        }
        if(y_neg){
          Corpoint1 = sourcePt + egoR*Eigen::Vector2d(extend_length[2],extend_length[3]);
          Corpoint2 = sourcePt + egoR*Eigen::Vector2d(extend_length[2],extend_length[3]-resolution);
          Corpoint3 = sourcePt + egoR*Eigen::Vector2d(extend_length[0],extend_length[3]-resolution);
          Corpoint4 = sourcePt + egoR*Eigen::Vector2d(extend_length[0],extend_length[3]);
          if(IfCollision_LineWithHeight(Corpoint1,Corpoint2) || IfCollision_LineWithHeight(Corpoint2,Corpoint3) || IfCollision_LineWithHeight(Corpoint3,Corpoint4)|| -(extend_length[3]-resolution)>range){
            y_neg = false;
          }
          else{
            extend_length[3]-=resolution;
          }
        }
      }
      // 要求h0x+h1y+h2z+h3<0
      double dir, sgn;
      Corpoint1 = sourcePt + egoR*Eigen::Vector2d(extend_length[0],extend_length[1]);
      Corpoint2 = sourcePt + egoR*Eigen::Vector2d(extend_length[0],extend_length[3]);
      Corpoint3 = sourcePt + egoR*Eigen::Vector2d(extend_length[2],extend_length[3]);
      Corpoint4 = sourcePt + egoR*Eigen::Vector2d(extend_length[2],extend_length[1]);
      // 右边界
      dir = sourcePt.x()*(Corpoint2.y()-Corpoint1.y()) + sourcePt.y()*(Corpoint1.x()-Corpoint2.x())+Corpoint1.y()*Corpoint2.x()-Corpoint1.x()*Corpoint2.y();
      sgn = -dir/std::abs(dir);
      //列向量储存 同hzc一样
      hPoly(0,0) = sgn*(Corpoint2.y()-Corpoint1.y());
      hPoly(1,0) = sgn*(Corpoint1.x()-Corpoint2.x());
      hPoly(2,0) = 0;
      hPoly(3,0) = sgn*(Corpoint1.y()*Corpoint2.x() - Corpoint1.x()*Corpoint2.y());
      hPoly.col(0) = hPoly.col(0)/hPoly.col(0).head(3).norm();
      // 下边界
      dir = sourcePt.x()*(Corpoint3.y()-Corpoint2.y()) + sourcePt.y()*(Corpoint2.x()-Corpoint3.x())+Corpoint2.y()*Corpoint3.x()-Corpoint2.x()*Corpoint3.y();
      sgn = -dir/std::abs(dir);
      hPoly(0,1) = sgn*(Corpoint3.y()-Corpoint2.y());
      hPoly(1,1) = sgn*(Corpoint2.x()-Corpoint3.x());
      hPoly(2,1) = 0;
      hPoly(3,1) = sgn*(Corpoint2.y()*Corpoint3.x() - Corpoint2.x()*Corpoint3.y());
      hPoly.col(1) = hPoly.col(1)/hPoly.col(1).head(3).norm();
      // 左边界
      dir = sourcePt.x()*(Corpoint4.y()-Corpoint3.y()) + sourcePt.y()*(Corpoint3.x()-Corpoint4.x())+Corpoint3.y()*Corpoint4.x()-Corpoint3.x()*Corpoint4.y();
      sgn = -dir/std::abs(dir);
      hPoly(0,2) = sgn*(Corpoint4.y()-Corpoint3.y());
      hPoly(1,2) = sgn*(Corpoint3.x()-Corpoint4.x());
      hPoly(2,2) = 0;
      hPoly(3,2) = sgn*(Corpoint3.y()*Corpoint4.x() - Corpoint3.x()*Corpoint4.y());
      hPoly.col(2) = hPoly.col(2)/hPoly.col(2).head(3).norm();
      // 上边界
      dir = sourcePt.x()*(Corpoint1.y()-Corpoint4.y()) + sourcePt.y()*(Corpoint4.x()-Corpoint1.x())+Corpoint4.y()*Corpoint1.x()-Corpoint4.x()*Corpoint1.y();
      sgn = -dir/std::abs(dir);
      hPoly(0,3) = sgn*(Corpoint1.y()-Corpoint4.y());
      hPoly(1,3) = sgn*(Corpoint4.x()-Corpoint1.x());
      hPoly(2,3) = 0;
      hPoly(3,3) = sgn*(Corpoint4.y()*Corpoint1.x() - Corpoint4.x()*Corpoint1.y());
      hPoly.col(3) = hPoly.col(3)/hPoly.col(3).head(3).norm();
      hPolys.push_back(hPoly);
    }


  }

  // 机臂
  void getManiCorridor(const std::vector<Eigen::Vector3d> &statelist, std::vector<Eigen::MatrixXd> &leftHPolys, std::vector<Eigen::MatrixXd> &rightHPolys){
    
    int state_num = statelist.size();
    leftHPolys.clear();
    rightHPolys.clear();
    // const double eps = 1.0e-6; 

    Eigen::MatrixX4d hp, gap;

    std::vector<Eigen::Vector3d> valid_pc;
    valid_pc.reserve(map_->surf_.size());

    Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
    bd(0, 0) = 1.0;
    bd(1, 0) = -1.0;
    bd(2, 1) = 1.0;
    bd(3, 1) = -1.0;
    bd(4, 2) = 1.0;
    bd(5, 2) = -1.0;
    for(int i=0; i<state_num; i++){
      Eigen::Vector2d sourcePt = statelist[i].head(2);
      double yaw = statelist[i][2];
      Eigen::Matrix3d egoR;
      egoR<<cos(yaw),-sin(yaw),0,sin(yaw),cos(yaw),0,0,0,1;


      Eigen::Vector3d a = Eigen::Vector3d(sourcePt.x(),sourcePt.y(),0) + egoR * Eigen::Vector3d(config_.length_/2-config_.rear_suspension_,-0.125,0.3);
      Eigen::Vector3d b = Eigen::Vector3d(sourcePt.x(),sourcePt.y(),0) + egoR * Eigen::Vector3d(config_.length_/2-config_.rear_suspension_,-0.125,0.45);
      bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, map_->x_upper);
      bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, map_->x_lower);
      bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, map_->y_upper);
      bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, map_->y_lower);
      bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, 0.5);
      bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, 0.25);
      valid_pc.clear();
      for (const Eigen::Vector3d &p : map_->surf_){
        // p在a b最大坐标+range 和 最小坐标-range内，则将这个点放进valid_pc
        if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0){
          valid_pc.emplace_back(p);
        }
      }
      Eigen::MatrixXd hpp;
      if( valid_pc.size()==0){
        hpp = bd.transpose();
      }
      else{
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pcr(valid_pc[0].data(), 3, valid_pc.size());
        firi::firi(bd, pcr, a, b, hp);
        hpp = hp.transpose();
      }
      for(size_t i=0; i<hpp.cols(); i++){
        hpp.col(i) = hpp.col(i)/hpp.col(i).head(3).norm();
      }
      rightHPolys.push_back(hpp);


      a = Eigen::Vector3d(sourcePt.x(),sourcePt.y(),0) + egoR * Eigen::Vector3d(config_.length_/2-config_.rear_suspension_,0.125,0.3);
      b = Eigen::Vector3d(sourcePt.x(),sourcePt.y(),0) + egoR * Eigen::Vector3d(config_.length_/2-config_.rear_suspension_,0.125,0.45);
      bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, map_->x_upper);
      bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, map_->x_lower);
      bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, map_->y_upper);
      bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, map_->y_lower);
      bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, 0.5);
      bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, 0.25);

      valid_pc.clear();
      for (const Eigen::Vector3d &p : map_->surf_){
        // p在a b最大坐标+range 和 最小坐标-range内，则将这个点放进valid_pc
        if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0){
          valid_pc.emplace_back(p);
        }
      }
      if( valid_pc.size()==0){
        hpp = bd.transpose();
      }
      else{
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pcr(valid_pc[0].data(), 3, valid_pc.size());
        firi::firi(bd, pcr, a, b, hp);
        hpp = hp.transpose();
      }

      for(size_t i=0; i<hpp.cols(); i++){
        hpp.col(i) = hpp.col(i)/hpp.col(i).head(3).norm();
      }
      leftHPolys.push_back(hpp);
    }
    

  }


  bool IfCollision_LineWithHeight(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2){
    double grid_interval = map_->grid_interval_;
    double z_min = grid_interval / 2;
    double z_max = (floor(config_.height_/grid_interval)-1)*grid_interval + z_min + 1e-3;
    Eigen::Vector2d vector12 = (point2-point1).normalized()*grid_interval/2;
    uint8_t check_occ;
    Eigen::Vector2d pos;
    for(pos = point1; (point2-pos).dot(vector12)>0; pos+=vector12){
      for(double z = z_min; z<z_max; z+=grid_interval){
        check_occ = map_->CheckCollisionBycoord(pos.x(),pos.y(),z);
        if(check_occ != 0){
          return true;
        }
      }
    }
    pos = point2;
    for(double z = z_min; z<z_max; z+=grid_interval){
      check_occ = map_->CheckCollisionBycoord(pos.x(),pos.y(),z);
      if(check_occ != 0){
        return true;
      }
    }
    return false;
  }


  template <typename EIGENVEC>
  inline void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  inline void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }


  static double costFunctionCallback(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g){
    Gcopter &obj = *(Gcopter *)ptr;
    obj.iter_num_ += 1;

    std::vector<Eigen::Map<const Eigen::MatrixXd>> P_container;
    std::vector<Eigen::Map<Eigen::MatrixXd>> gradP_container;


    g.setZero();
    // 将输入变量映射到变量矩阵
    int offset = 0;
    for(int trajid = 0; trajid < obj.piece_singul_num_; trajid++){
      Eigen::Map<const Eigen::MatrixXd> P(x.data()+offset, 2, obj.eachTrajNums[trajid] - 1);
      Eigen::Map<Eigen::MatrixXd>gradP(g.data()+offset, 2, obj.eachTrajNums[trajid] - 1);
      offset += 2 * (obj.eachTrajNums[trajid] - 1);
      gradP.setZero();
      P_container.push_back(P);
      gradP_container.push_back(gradP);
    }

    Eigen::Map<const Eigen::VectorXd> t(x.data()+offset, obj.piece_singul_num_);
    Eigen::Map<Eigen::VectorXd> gradt(g.data()+offset, obj.piece_singul_num_);
    Eigen::VectorXd T(obj.piece_singul_num_);

    // 这里的T是实际的T，且是这一段总共的T，因此在arrayT里面需要分成很多部分
    obj.VirtualT2RealT(t, T);

    std::vector<Eigen::VectorXd> arrayt_container;
    std::vector<Eigen::VectorXd> arraygradt_container;
    // 储存所有小段T的梯度
    // std::vector<Eigen::VectorXd> arraygradT_container;
    // 储存所有小段T
    std::vector<double> pieceT_container;
    
    for(int trajid = 0; trajid < obj.piece_singul_num_; trajid++){
      // Eigen::VectorXd arraygradT(obj.eachTrajNums[trajid]);
      double pieceT;
      pieceT = T[trajid]/obj.eachTrajNums[trajid];
      // arraygradT.setZero();
      pieceT_container.push_back(pieceT);
      // arraygradT_container.push_back(arraygradT);
    }
    double costOfAll = 0;
// std::cout<<"-------------------------------------------------------------"<<std::endl;
    for(int trajid = 0; trajid < obj.piece_singul_num_; trajid++){
      // obj.partialGradByCoeffs.resize(6*obj.eachTrajNums[trajid],obj.Freedom_);
      // obj.partialGradByTimes.resize(obj.eachTrajNums[trajid],obj.Freedom_);
      double cost;
      obj.mincos[trajid].setParameters(P_container[trajid],pieceT_container[trajid]);
      obj.mincos[trajid].getEnergy(cost);
      obj.mincos[trajid].getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
      obj.mincos[trajid].getEnergyPartialGradByTimes(obj.partialGradByTimes);
      obj.attachPenaltyFunctional(trajid,cost);
      obj.mincos[trajid].propogateGrad(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                       obj.gradByPoints, obj.gradByTimes);
      cost += obj.rho * T[trajid];

      gradP_container[trajid] = obj.gradByPoints;

      obj.gradOneTime = obj.gradByTimes.sum()/obj.eachTrajNums[trajid] + obj.rho;
      backwardGradT(t[trajid], obj.gradOneTime, gradt[trajid]);

      costOfAll += cost;
    }
// std::cout<<"cost: "<<costOfAll<<std::endl;
// std::cout<<"x: "<<x.transpose()<<std::endl;
// std::cout<<"g: "<<g.transpose()<<std::endl;
    
    return costOfAll;
  }

  // 梯度给partialGradByCoeffs和partialGradByTimes
  void attachPenaltyFunctional(const int &trajid, double &cost){
    int N = eachTrajNums[trajid];

    Eigen::Vector2d outerNormal;
    Eigen::Vector2d sigma, dsigma, ddsigma, dddsigma;
    double vel2_reci,vel2_reci_e,vel3_2_reci_e,acc2, cur2, cur;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 2> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaKc,gradViolaKLc,gradViolaKRc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaKt,gradViolaKLt,gradViolaKRt;
    double violaPos, violaVel, violaAcc, violaCur, violaCurL, violaCurR, violaDynamicObs;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaCurPenaD, violaCurPenaDL, violaCurPenaDR,violaDynamicObsPenaD;
    double violaPosPena, violaVelPena, violaAccPena, violaCurPena, violaCurPenaL, violaCurPenaR,violaDynamicObsPena;   

    double approxcur2, approxviolaCur,approxviolaCurPenaD,approxviolaCurPena;
    Eigen::Matrix<double, 6, 2> gradapproxViolaKc;
    double gradapproxViolaKt;

    Eigen::Matrix2d B_h;
    B_h << 0, -1,
           1,  0;
    double epis = 1.0e-8;

    std::vector<Eigen::MatrixXd> cfgHs = hPolyses[trajid];
    std::vector<Eigen::MatrixXd> leftcfgHs = leftManiHpolyses[trajid];
    std::vector<Eigen::MatrixXd> rightcfgHs = rightManiHpolyses[trajid];
    int singul_ = singuls[trajid];

    double omg;

    double z_h0, z_h1, z_h2, z_h3, z_h4;
    Eigen::Matrix2d ego_R, help_R;

    // int innerLoop;
    double T = mincos[trajid].getT1();
    int pointid = -1;


    double cost_corr=0, cost_v=0, cost_a=0, cost_cur=0, cost_omega = 0, cost_domega = 0;
    for (int i = 0; i < N; ++i){
      int K;
      if(i==0 || i==N-1)
        K = config_.denseResolution_;
      else{
        K = config_.sparseResolution_;
      }
      const Eigen::Matrix<double,6,2> &c = mincos[trajid].getCoeffs().block<6,2>(6*i,0);
      step = T / K;

      s1 = 0.0;

      for (int j = 0; j <= K; ++j){
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0, 24.0, 120.0 * s1;
        alpha = 1.0 / K * j;

        //update s1 for the next iteration
        s1 += step;
        pointid++;

        sigma = c.transpose() * beta0;
        dsigma = c.transpose() * beta1;
        ddsigma = c.transpose() * beta2;
        dddsigma = c.transpose() * beta3;
        Eigen::Vector2d ddddsigma = c.transpose() * beta4;


        // ctrl_points_.col(i_dp) = sigma;
        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        // some help values
        
        z_h0 = dsigma.norm();
        z_h1 = ddsigma.transpose() * dsigma;
        z_h2 = dddsigma.transpose() * dsigma;
        z_h3 = ddsigma.transpose() * B_h * dsigma;

        // add cost z_h0 = ||v||
        if ( z_h0 < 1e-4 || (j==0&&i==0) || (i==N-1&&j==K)){
          continue;
        }
        //avoid siguality
        vel2_reci = 1.0 / (z_h0 * z_h0);
        vel2_reci_e = 1.0 / (z_h0 * z_h0+epis);
        vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
        z_h0 = 1.0 / z_h0;

        z_h4 = z_h1 * vel2_reci;
        violaVel = 1.0 / vel2_reci - config_.max_vel_ * config_.max_vel_;
        acc2 = z_h1 * z_h1 * vel2_reci;
        cur2 = z_h3 * z_h3 * (vel2_reci_e * vel2_reci_e * vel2_reci_e);
        cur = z_h3 * vel3_2_reci_e;
        violaAcc = acc2 - config_.max_acc_ * config_.max_acc_;
        
        violaCur = cur2 - config_.max_cur_ * config_.max_cur_;
        violaCurL = cur - config_.max_cur_;
        violaCurR = -cur - config_.max_cur_;
        // ROS_INFO("cur: %f, violaCurL: %f, violaCurR: %f",cur,violaCurL,violaCurR);
        // zmk
        double omega = z_h3 * vel2_reci_e;
        double max_omega_ = 2*(config_.max_vel_- dsigma.norm())/config_.wheel_base_;
        double violaOmegaL = omega - max_omega_;
        double violaOmegaR = -omega - max_omega_;
        // std::cout<<config_.max_vel_<<"      "<< dsigma.norm()<<"            "<<config_.wheel_base_<<std::endl;
        // std::cout<<"omega:"<<omega<<"  max_omega_:"<<max_omega_<<"    violaOmegaL:"<<violaOmegaL<<"    violaOmegaR:"<<violaOmegaR<<std::endl;
        double z_h4 = dddsigma.transpose() * B_h * dsigma;
        double z_h5 = dddsigma.transpose() * B_h * ddsigma;
        double z_h6 = ddddsigma.transpose() * B_h * dsigma;
        double domega = z_h4*vel2_reci_e - 2.0 * z_h3 * z_h1 * vel2_reci_e * vel2_reci_e;
        double violadOmegaL = domega - config_.max_domega_;
        double violadOmegaR = -domega - config_.max_domega_;

        int singul = singuls[trajid];

        ego_R << dsigma(0), -dsigma(1),
                 dsigma(1),  dsigma(0);
        ego_R = ego_R * z_h0;

        help_R << ddsigma(0), -ddsigma(1),
                  ddsigma(1),  ddsigma(0);
        help_R = help_R * z_h0 ;

        // 车体
        for (auto cp : check_point){
         
          cp = cp*singul;
          Eigen::Vector2d bpt = sigma + ego_R * cp;
          Eigen::Matrix2d help_L;
          help_L << cp(0), -cp(1),
                    cp(1),  cp(0);
          int corr_k = cfgHs[pointid].cols();
          for (int k = 0; k < corr_k; k++){
            outerNormal = cfgHs[pointid].col(k).head<2>();
            // violaPos = outerNormal.dot(bpt - cfgHs[pointid].col(k).tail<2>());
            violaPos = outerNormal.dot(bpt) + cfgHs[pointid].col(k)[3] + safeDis;
            if (violaPos > 0.0){
              positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);

              gradViolaPc = beta0 * outerNormal.transpose() +
                            beta1 * outerNormal.transpose() * (help_L * z_h0 - ego_R * cp * dsigma.transpose() * vel2_reci);
              gradViolaPt = alpha * outerNormal.transpose() * (dsigma + help_R * cp - ego_R * z_h1 * cp * vel2_reci);
              
              partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[4] * violaPosPenaD * gradViolaPc;
              partialGradByTimes(i) += omg * penaltyWt[4] * (violaPosPenaD * gradViolaPt * step + violaPosPena / K);
              cost += omg * step * penaltyWt[4] * violaPosPena; // cost is the same
              cost_corr += omg * step * penaltyWt[4] * violaPosPena; 
            }
          }
        }

        // 左机臂
        for (auto cp : leftMani_check_point){        
          cp.head(2) = cp.head(2)*singul;
          Eigen::Vector2d bpt = sigma + ego_R * cp.head(2);
          Eigen::Matrix2d help_L;
          help_L << cp(0), -cp(1),
                    cp(1),  cp(0);
          int corr_k = leftcfgHs[pointid].cols();
          for (int k = 0; k < corr_k; k++){
            outerNormal = leftcfgHs[pointid].col(k).head<2>();
            if(abs(leftcfgHs[pointid].col(k)[2])>0.01){
              continue;
            }
            // double sinAz = sqrt(1-leftcfgHs[pointid].col(k)[2]*leftcfgHs[pointid].col(k)[2]);
            violaPos = (outerNormal.dot(bpt) + leftcfgHs[pointid].col(k)[2]*cp.z() + leftcfgHs[pointid].col(k)[3]) + safeDis/3;
            // violaPos = violaPos*sinAz;
            if (violaPos > 0.0){
              // ROS_INFO("leftMani_check_point!!!! %f,%f,%f",bpt.x(),bpt.y(),cp.z());   
              positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
              gradViolaPc = beta0 * outerNormal.transpose() +
                            beta1 * outerNormal.transpose() * (help_L * z_h0 - ego_R * cp.head(2) * dsigma.transpose() * vel2_reci);
              // gradViolaPc = sinAz * gradViolaPc;
              gradViolaPt = alpha * outerNormal.transpose() * (dsigma + help_R * cp.head(2) - ego_R * z_h1 * cp.head(2) * vel2_reci);
              // gradViolaPt = sinAz * gradViolaPt;
              partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[4]*100 * violaPosPenaD * gradViolaPc;
              partialGradByTimes(i) += omg * penaltyWt[4]*100 * (violaPosPenaD * gradViolaPt * step + violaPosPena / K);
              cost += omg * step * penaltyWt[4]*100 * violaPosPena; // cost is the same
              cost_corr += omg * step * penaltyWt[4] * violaPosPena; 
            }
          }
        }
        // 右机臂
        for (auto cp : rightMani_check_point){     
          cp.head(2) = cp.head(2)*singul;
          Eigen::Vector2d bpt = sigma + ego_R * cp.head(2);
          Eigen::Matrix2d help_L;
          help_L << cp(0), -cp(1),
                    cp(1),  cp(0);
          int corr_k = rightcfgHs[pointid].cols();
          for (int k = 0; k < corr_k; k++){
            outerNormal = rightcfgHs[pointid].col(k).head<2>();
            if(abs(rightcfgHs[pointid].col(k)[2]) > 0.01){
              continue;
            }
            // double sinAz = sqrt(1-rightcfgHs[pointid].col(k)[2]*rightcfgHs[pointid].col(k)[2]);
            violaPos = (outerNormal.dot(bpt) + rightcfgHs[pointid].col(k)[2]*cp.z() + rightcfgHs[pointid].col(k)[3]) + safeDis/3;
            if (violaPos > 0.0){
              // ROS_INFO("rightMani_check_point!!!! %f,%f,%f",bpt.x(),bpt.y(),cp.z());   
              positiveSmoothedL1(violaPos, violaPosPena, violaPosPenaD);
              gradViolaPc = beta0 * outerNormal.transpose() +
                            beta1 * outerNormal.transpose() * (help_L * z_h0 - ego_R * cp.head(2) * dsigma.transpose() * vel2_reci);
              // gradViolaPc = sinAz * gradViolaPc;              
              gradViolaPt = alpha * outerNormal.transpose() * (dsigma + help_R * cp.head(2) - ego_R * z_h1 * cp.head(2) * vel2_reci);
              // gradViolaPt = sinAz * gradViolaPt;
              partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[4]*100 * violaPosPenaD * gradViolaPc;
              partialGradByTimes(i) += omg * penaltyWt[4]*100 * (violaPosPenaD * gradViolaPt * step + violaPosPena / K);
              cost += omg * step * penaltyWt[4]*100 * violaPosPena; // cost is the same
              cost_corr += omg * step * penaltyWt[4] * violaPosPena; 
            }
          }
        }





        double gradt, grad_prev_t, costp;
        Eigen::Vector2d gradp, gradp2;

        if (violaVel > 0.0){
          positiveSmoothedL1(violaVel, violaVelPena, violaVelPenaD);

          gradViolaVc = 2.0 * beta1 * dsigma.transpose(); // 6*2
          gradViolaVt = 2.0 * alpha * z_h1;               // 1*1
          partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[0] * violaVelPenaD * gradViolaVc;
          partialGradByTimes(i) += omg * penaltyWt[0] * (violaVelPenaD * gradViolaVt * step + violaVelPena / K);
          cost += omg * step * penaltyWt[0] * violaVelPena;
          cost_v +=omg * step * penaltyWt[0] * violaVelPena;
        }

        // cost_a = 0;
        if (violaAcc > 0.0){
          positiveSmoothedL1(violaAcc, violaAccPena, violaAccPenaD);
          gradViolaAc = 2.0 * beta1 * (z_h4 * ddsigma.transpose() - z_h4 * z_h4 * dsigma.transpose()) +
                        2.0 * beta2 * z_h4 * dsigma.transpose(); // 6*2
          gradViolaAt = 2.0 * alpha * (z_h4 * (ddsigma.squaredNorm() + z_h2) - z_h4 * z_h4 * z_h1);
          partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[1] * violaAccPenaD * gradViolaAc;
          partialGradByTimes(i) += omg * penaltyWt[1] * (violaAccPenaD * gradViolaAt * step + violaAccPena / K);
          cost += omg * step * penaltyWt[1] * violaAccPena;
          cost_a += omg * step * penaltyWt[1] * violaAccPena;
        } 
        
        // if(violaCurL > 0.0 ){
        //   positiveSmoothedL1(violaCurL, violaCurPenaL, violaCurPenaDL);
        //   //@hzc
        //   gradViolaKLc = beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) 
        //                  + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose(); // 6*2
        //   gradViolaKLt  = alpha*vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1);
        //   partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[2] * violaCurPenaDL * gradViolaKLc;
        //   partialGradByTimes(i) += omg * penaltyWt[2] * (violaCurPenaDL * gradViolaKLt * step + violaCurPenaL / K);
        //   cost += omg * step * penaltyWt[2] * violaCurPenaL;
        //   cost_cur +=omg * step * penaltyWt[2] * violaCurPenaL;
        // }
        // if(violaCurR > 0.0){
        //   positiveSmoothedL1(violaCurR, violaCurPenaR, violaCurPenaDR);
        //   //@hzc
        //   gradViolaKRc = -(beta1 * (vel3_2_reci_e * ddsigma.transpose()*B_h - 3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) 
        //                  + beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose()); // 6*2
        //   gradViolaKRt  = -(alpha*vel3_2_reci_e*(dddsigma.transpose()*B_h*dsigma-3*vel2_reci_e*z_h3*z_h1));
        //   partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[2] * violaCurPenaDR * gradViolaKRc;
        //   partialGradByTimes(i) += omg * penaltyWt[2] * (violaCurPenaDR * gradViolaKRt * step + violaCurPenaR / K);
        //   cost += omg * step * penaltyWt[2] * violaCurPenaR;
        //   cost_cur +=omg * step * penaltyWt[2] * violaCurPenaR;
        // }
        
        double violaOmegaPenaL,violaOmegaPenaDL,violaOmegaPenaR,violaOmegaPenaDR;
        if(violaOmegaL > 0.0){
          positiveSmoothedL1(violaOmegaL, violaOmegaPenaL, violaOmegaPenaDL);
          //zmk
          gradViolaKLc = beta1 * (vel2_reci_e * ddsigma.transpose()*B_h - 2 * vel2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose() + dsigma.transpose() * sqrt(vel2_reci_e)/config_.wheel_base_) 
                         + beta2 * vel2_reci_e * dsigma.transpose() * B_h.transpose(); // 6*2
          gradViolaKLt  = alpha*( vel2_reci_e*(dddsigma.transpose()*B_h*dsigma - 2*vel2_reci_e*z_h3*z_h1) + dsigma.transpose() * sqrt(vel2_reci_e)/config_.wheel_base_ * ddsigma);
          partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[3] * violaOmegaPenaDL * gradViolaKLc;
          partialGradByTimes(i) += omg * penaltyWt[3] * (violaOmegaPenaDL * gradViolaKLt * step + violaOmegaPenaL / K);
          cost += omg * step * penaltyWt[3] * violaOmegaPenaL;
          cost_omega +=omg * step * penaltyWt[3] * violaOmegaPenaL;
          // std::cout<<"OmegaL: "<<omega<<"           maxomega:"<<max_omega_<< "      cost:"<<omg * step * penaltyWt[3] * violaOmegaPenaL<<std::endl;
        }
        if(violaOmegaR > 0.0){
          positiveSmoothedL1(violaOmegaR, violaOmegaPenaR, violaOmegaPenaDR);
          //zmk
          gradViolaKRc = -(beta1 * (vel2_reci_e * ddsigma.transpose()*B_h - 2 * vel2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose() - dsigma.transpose() * sqrt(vel2_reci_e)/config_.wheel_base_) 
                           + beta2 * vel2_reci_e * dsigma.transpose() * B_h.transpose()); // 6*2
          gradViolaKRt  = -(alpha*( vel2_reci_e*(dddsigma.transpose()*B_h*dsigma - 2*vel2_reci_e*z_h3*z_h1) - dsigma.transpose() * sqrt(vel2_reci_e)/config_.wheel_base_ * ddsigma ));
          partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[3] * violaOmegaPenaDR * gradViolaKRc;
          partialGradByTimes(i) += omg * penaltyWt[3] * (violaOmegaPenaDR * gradViolaKRt * step + violaOmegaPenaR / K);
          cost += omg * step * penaltyWt[3] * violaOmegaPenaR;
          cost_omega += omg * step * penaltyWt[3] * violaOmegaPenaR;
          // std::cout<<"OmegaR: "<<omega<<"           maxomega:"<<max_omega_<< "      cost:"<<omg * step * penaltyWt[3] * violaOmegaPenaL<<std::endl;
        }

        double violadOmegaPenaL,violadOmegaPenaDL,violadOmegaPenaR,violadOmegaPenaDR;
        if(violadOmegaL > 0.0){
          positiveSmoothedL1(violadOmegaL, violadOmegaPenaL, violadOmegaPenaDL);
          //zmk
          gradViolaKLc = beta3 * (dsigma.transpose()*B_h.transpose() * vel2_reci_e) +
                         -beta2 * 2.0 * vel2_reci_e * vel2_reci_e * ( z_h1 * dsigma.transpose()*B_h.transpose() + z_h3 * dsigma.transpose()) +
                         beta1 * ((dddsigma.transpose() * B_h * vel2_reci_e) 
                                  - vel2_reci_e * vel2_reci_e * 2.0 * ( z_h4 * dsigma.transpose() +
                                                                        z_h1 * ddsigma.transpose()*B_h + z_h3 * ddsigma.transpose())
                                  + 8.0 * z_h3 * z_h1 * dsigma.transpose() * vel2_reci_e * vel2_reci_e * vel2_reci_e);
          gradViolaKLt = alpha * ((z_h5 + z_h6) * vel2_reci_e + 
                                  -(ddsigma.squaredNorm()*z_h3 + z_h2*z_h3 + z_h4*2.0*z_h1) * 2.0  * vel2_reci_e * vel2_reci_e + 
                                  8.0 * z_h3 * z_h1 * z_h1 * vel2_reci_e * vel2_reci_e * vel2_reci_e);
          partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[5] * violadOmegaPenaDL * gradViolaKLc;
          partialGradByTimes(i) += omg * penaltyWt[5] * (violadOmegaPenaDL * gradViolaKLt * step + violadOmegaPenaL / K);
          cost += omg * step * penaltyWt[5] * violadOmegaPenaL;
          cost_domega +=omg * step * penaltyWt[5] * violadOmegaPenaL;
          // std::cout<<"OmegaL: "<<omega<<"           maxomega:"<<max_omega_<< "      cost:"<<omg * step * penaltyWt[3] * violaOmegaPenaL<<std::endl;
        }
        if(violadOmegaR > 0.0){
          positiveSmoothedL1(violadOmegaR, violadOmegaPenaR, violadOmegaPenaDR);
          //zmk
          gradViolaKRc = -beta3 * (dsigma.transpose()*B_h.transpose() * vel2_reci_e) +
                         beta2 * 2.0 * vel2_reci_e * vel2_reci_e * ( z_h1 * dsigma.transpose()*B_h.transpose() + z_h3 * dsigma.transpose()) +
                         -beta1 * ((dddsigma.transpose() * B_h * vel2_reci_e) 
                                  - vel2_reci_e * vel2_reci_e * 2.0 * ( z_h4 * dsigma.transpose() +
                                                                        z_h1 * ddsigma.transpose()*B_h + z_h3 * ddsigma.transpose())
                                  + 8.0 * z_h3 * z_h1 * dsigma.transpose() * vel2_reci_e * vel2_reci_e * vel2_reci_e);
          gradViolaKRt = -alpha * ((z_h5 + z_h6) * vel2_reci_e + 
                                  -(ddsigma.squaredNorm()*z_h3 + z_h2*z_h3 + z_h4*2.0*z_h1) * 2.0  * vel2_reci_e * vel2_reci_e + 
                                  8.0 * z_h3 * z_h1 * z_h1 * vel2_reci_e * vel2_reci_e * vel2_reci_e);
          partialGradByCoeffs.block<6,2>(i*6 , 0) += omg * step * penaltyWt[5] * violadOmegaPenaDR * gradViolaKRc;
          partialGradByTimes(i) += omg * penaltyWt[5] * (violadOmegaPenaDR * gradViolaKRt * step + violadOmegaPenaR / K);
          cost += omg * step * penaltyWt[5] * violadOmegaPenaR;
          cost_domega +=omg * step * penaltyWt[5] * violadOmegaPenaR;
          // std::cout<<"OmegaL: "<<omega<<"           maxomega:"<<max_omega_<< "      cost:"<<omg * step * penaltyWt[3] * violaOmegaPenaL<<std::endl;
        }
      }

    }

    if(ifprint){
      std::cout<<"cost corridor: "<< cost_corr<<std::endl;
      std::cout<<"cost v: "<< cost_v<<std::endl;
      std::cout<<"cost a: "<< cost_a<<std::endl;
      std::cout<<"cost cur: "<< cost_cur<<std::endl;
      std::cout<<"cost omega: "<< cost_omega<<std::endl;
      std::cout<<"cost domega: "<< cost_domega<<std::endl;
      ifprint = false;
    }


  }

  inline void positiveSmoothedL1(const double &x, double &f, double &df){
    const double pe = smoothEps;
    const double half = 0.5 * pe;
    const double f3c = 1.0 / (pe * pe);
    const double f4c = -0.5 * f3c / pe;
    const double d2c = 3.0 * f3c;
    const double d3c = 4.0 * f4c;

    if (x < pe){
        f = (f4c * x + f3c) * x * x * x;
        df = (d3c * x + d2c) * x * x;
    }
    else{
        f = x - half;
        df = 1.0;
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(const double &tau,
                                   const double &gradT,
                                   EIGENVEC &gradTau){
    double gradrt2vt;
    if(tau>0){
      gradrt2vt = tau+1.0;
    }
    else{
      double denSqrt = (0.5*tau-1.0)*tau+1.0;
      gradrt2vt = (1.0-tau)/(denSqrt*denSqrt);
    }
    gradTau = gradT * gradrt2vt;
  }

  // template <typename EIGENVEC>
  // static inline void backwardGradT(const double &tau,
  //                                  const double &gradT,
  //                                  EIGENVEC &gradTau){
  //   double gdVT2Rt;
  //   if (tau > 0)
  //   {
  //     gdVT2Rt = tau + 1.0;
  //   }
  //   else
  //   {
  //     double denSqrt = (0.5 * tau - 1.0) * tau + 1.0;
  //     gdVT2Rt = (1.0 - tau) / (denSqrt * denSqrt);
  //   }

  //   gradTau = gradT * gdVT2Rt;
  // }

  void vis_2Dbox(){
    visualization_msgs::Marker rec;

    rec.type = visualization_msgs::Marker::LINE_LIST;
    rec.pose.orientation.w = 1.00;
    rec.ns = "2Dbox";
    rec.color.r = rand() / double(RAND_MAX);
    rec.color.g = rand() / double(RAND_MAX);
    rec.color.b = rand() / double(RAND_MAX);
    rec.color.a = 0.50;
    rec.scale.x = 0.01;
    rec.header.frame_id = "base";
    rec.lifetime = ros::Duration();
    rec.header.stamp = ros::Time::now();

    rec.action = visualization_msgs::Marker::DELETEALL;
    box2DPub.publish(rec);

    rec.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point point1;
    geometry_msgs::Point point2;
    geometry_msgs::Point point3;
    geometry_msgs::Point point4;
    point1.z = 0;
    point2.z = 0;
    point3.z = 0;
    point4.z = 0;
    Eigen::MatrixXd curhPolys;
    for(int i = 0; i<hPolyses.size();i++){
      srand((unsigned int)(time(NULL))+i);
      rec.color.r = rand() / double(RAND_MAX);
      rec.color.g = rand() / double(RAND_MAX);
      rec.color.b = rand() / double(RAND_MAX);
      rec.points.clear();
      for(int j=0; j<hPolyses[i].size(); j++){
        rec.id = j*10+i;
        curhPolys = hPolyses[i][j].transpose();
        // 行向量形式算的
        point1.x =  (curhPolys(1,3)*curhPolys(0,1) - curhPolys(0,3)*curhPolys(1,1)) / (curhPolys(0,0)*curhPolys(1,1) - curhPolys(1,0)*curhPolys(0,1));
        point1.y =  (curhPolys(0,3)*curhPolys(1,0) - curhPolys(1,3)*curhPolys(0,0)) / (curhPolys(0,0)*curhPolys(1,1) - curhPolys(1,0)*curhPolys(0,1));
        point2.x =  (curhPolys(2,3)*curhPolys(1,1) - curhPolys(1,3)*curhPolys(2,1)) / (curhPolys(1,0)*curhPolys(2,1) - curhPolys(2,0)*curhPolys(1,1));
        point2.y =  (curhPolys(1,3)*curhPolys(2,0) - curhPolys(2,3)*curhPolys(1,0)) / (curhPolys(1,0)*curhPolys(2,1) - curhPolys(2,0)*curhPolys(1,1));
        point3.x =  (curhPolys(3,3)*curhPolys(2,1) - curhPolys(2,3)*curhPolys(3,1)) / (curhPolys(2,0)*curhPolys(3,1) - curhPolys(3,0)*curhPolys(2,1));
        point3.y =  (curhPolys(2,3)*curhPolys(3,0) - curhPolys(3,3)*curhPolys(2,0)) / (curhPolys(2,0)*curhPolys(3,1) - curhPolys(3,0)*curhPolys(2,1));
        point4.x =  (curhPolys(0,3)*curhPolys(3,1) - curhPolys(3,3)*curhPolys(0,1)) / (curhPolys(3,0)*curhPolys(0,1) - curhPolys(0,0)*curhPolys(3,1));
        point4.y =  (curhPolys(3,3)*curhPolys(0,0) - curhPolys(0,3)*curhPolys(3,0)) / (curhPolys(3,0)*curhPolys(0,1) - curhPolys(0,0)*curhPolys(3,1));
        
        rec.points.push_back(point1);
        rec.points.push_back(point2);
        rec.points.push_back(point2);
        rec.points.push_back(point3);
        rec.points.push_back(point3);
        rec.points.push_back(point4);
        rec.points.push_back(point4);
        rec.points.push_back(point1);
      }
      box2DPub.publish(rec);
    }


  }

  void vis_left3Dbox(){
    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    ROS_INFO("size of leftManiHpolyses[0]: %d, size of rightManiHpolyses[0]: %d", leftManiHpolyses[0].size(), rightManiHpolyses[0].size());
    for(size_t i=0; i<leftManiHpolyses.size();i++){
      for(size_t j=0; j<leftManiHpolyses[i].size(); j++){
        oldTris = mesh;
        Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
        geo_utils::enumerateVs(leftManiHpolyses[i][j].transpose(), vPoly);
        quickhull::QuickHull<double> tinyQH;
        const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
        const auto &idxBuffer = polyHull.getIndexBuffer();
        int hNum = idxBuffer.size() / 3;
        curTris.resize(3, hNum * 3);
        for (int i = 0; i < hNum * 3; i++){
          curTris.col(i) = vPoly.col(idxBuffer[i]);
        }
        mesh.resize(3, oldTris.cols() + curTris.cols());
        mesh.leftCols(oldTris.cols()) = oldTris;
        mesh.rightCols(curTris.cols()) = curTris;
      }
    }


    visualization_msgs::Marker meshMarker, edgeMarker;

    meshMarker.id = 0;
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = "base";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = "mesh";
    meshMarker.color.r = 0.00;
    meshMarker.color.g = 0.00;
    meshMarker.color.b = 1.00;
    meshMarker.color.a = 0.05;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 1.00;
    edgeMarker.color.g = 1.00;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 0.05;
    edgeMarker.scale.x = 0.005;

    geometry_msgs::Point point;

    int ptnum = mesh.cols();
    for (int i = 0; i < ptnum; i++)
    {
        point.x = mesh(0, i);
        point.y = mesh(1, i);
        point.z = mesh(2, i);
        meshMarker.points.push_back(point);
    }

    for (int i = 0; i < ptnum / 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            point.x = mesh(0, 3 * i + j);
            point.y = mesh(1, 3 * i + j);
            point.z = mesh(2, 3 * i + j);
            edgeMarker.points.push_back(point);
            point.x = mesh(0, 3 * i + (j + 1) % 3);
            point.y = mesh(1, 3 * i + (j + 1) % 3);
            point.z = mesh(2, 3 * i + (j + 1) % 3);
            edgeMarker.points.push_back(point);
        }
    }
    meshPub.publish(meshMarker);
    leftedgePub.publish(edgeMarker);
  }

  void vis_right3Dbox(){
    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);
    ROS_INFO("size of leftManiHpolyses[0]: %d, size of rightManiHpolyses[0]: %d", leftManiHpolyses[0].size(), rightManiHpolyses[0].size());
    for(size_t i=0; i<rightManiHpolyses.size();i++){
      for(size_t j=0; j<rightManiHpolyses[i].size(); j++){
        oldTris = mesh;
        Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
        geo_utils::enumerateVs(rightManiHpolyses[i][j].transpose(), vPoly);
        quickhull::QuickHull<double> tinyQH;
        const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
        const auto &idxBuffer = polyHull.getIndexBuffer();
        int hNum = idxBuffer.size() / 3;
        curTris.resize(3, hNum * 3);
        for (int i = 0; i < hNum * 3; i++){
          curTris.col(i) = vPoly.col(idxBuffer[i]);
        }
        mesh.resize(3, oldTris.cols() + curTris.cols());
        mesh.leftCols(oldTris.cols()) = oldTris;
        mesh.rightCols(curTris.cols()) = curTris;
      }
    }

    visualization_msgs::Marker meshMarker, edgeMarker;

    meshMarker.id = 1;
    meshMarker.header.stamp = ros::Time::now();
    meshMarker.header.frame_id = "base";
    meshMarker.pose.orientation.w = 1.00;
    meshMarker.action = visualization_msgs::Marker::ADD;
    meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    meshMarker.ns = "mesh";
    meshMarker.color.r = 0.00;
    meshMarker.color.g = 0.00;
    meshMarker.color.b = 1.00;
    meshMarker.color.a = 0.05;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 0.00;
    edgeMarker.color.g = 0.00;
    edgeMarker.color.b = 1.00;
    edgeMarker.color.a = 0.2;
    edgeMarker.scale.x = 0.005;

    geometry_msgs::Point point;

    int ptnum = mesh.cols();
    for (int i = 0; i < ptnum; i++)
    {
        point.x = mesh(0, i);
        point.y = mesh(1, i);
        point.z = mesh(2, i);
        meshMarker.points.push_back(point);
    }

    for (int i = 0; i < ptnum / 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            point.x = mesh(0, 3 * i + j);
            point.y = mesh(1, 3 * i + j);
            point.z = mesh(2, 3 * i + j);
            edgeMarker.points.push_back(point);
            point.x = mesh(0, 3 * i + (j + 1) % 3);
            point.y = mesh(1, 3 * i + (j + 1) % 3);
            point.z = mesh(2, 3 * i + (j + 1) % 3);
            edgeMarker.points.push_back(point);
        }
    }
    meshPub.publish(meshMarker);
    rightedgePub.publish(edgeMarker);

  }

  void vis_innerPoint(){
    // ROS_INFO("\033[41;37m  innerpoints.size(): %d  \033[0m",innerpoints.size());  
    visualization_msgs::MarkerArray markerarraydelete;
    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "base";
    marker.ns = "innerPoint";
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::CYLINDER;

    marker.action = visualization_msgs::Marker::DELETEALL;
    markerarraydelete.markers.push_back(marker);
    point_pub.publish(markerarraydelete);
    
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.04;
    marker.color.a = 0.8;
    marker.color.r = 1.0-195.0/255;
    marker.color.g = 1.0-176.0/255;
    marker.color.b = 1.0-145.0/255;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.position.z = 0.15;

    for(int i=0; i<Innerpointses.size();i++){
      for(int j=0;j<Innerpointses[i].cols();j++){
        marker.header.stamp = ros::Time::now();
        marker.id = j*100+i;
        marker.pose.position.x = Innerpointses[i].col(j).x();
        marker.pose.position.y = Innerpointses[i].col(j).y();
        markerarray.markers.push_back(marker);
      }
    }
    point_pub.publish(markerarray);
  }
  
  void vis_finalinnerPoint(){
    // ROS_INFO("\033[41;37m  innerpoints.size(): %d  \033[0m",innerpoints.size());  
    visualization_msgs::MarkerArray markerarraydelete;
    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "base";
    marker.ns = "innerPoint";
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::CYLINDER;
    
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.12;
    marker.scale.y = 0.12;
    marker.scale.z = 0.04;
    marker.color.a = 0.8;
    marker.color.r = 95.0/255;
    marker.color.g = 76.0/255;
    marker.color.b = 45.0/255;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.position.z = 0.15;

    for(int i=0; i<finalInnerpointses.size();i++){
      for(int j=0;j<finalInnerpointses[i].cols();j++){
        marker.header.stamp = ros::Time::now();
        marker.id = j*10000+i*100;
        marker.pose.position.x = finalInnerpointses[i].col(j).x();
        marker.pose.position.y = finalInnerpointses[i].col(j).y();
        markerarray.markers.push_back(marker);
      }
    }
    point_pub.publish(markerarray);
  }

};


#endif