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


struct Config
{
    // std::string mapTopic;
    // std::string targetTopic;
    // double dilateRadius;
    // double voxelWidth;
    // std::vector<double> mapBound;
    // double timeoutRRT;
    // double maxVelMag;
    // double maxBdrMag;
    // double maxTiltAngle;
    // double minThrust;
    // double maxThrust;
    // double vehicleMass;
    // double gravAcc;
    // double horizDrag;
    // double vertDrag;
    // double parasDrag;
    // double speedEps;
    // double weightT;
    // std::vector<double> chiVec;
    // double smoothingEps;
    // int integralIntervs;
    // double relCostTol;

    
    double max_vel_;
    double max_acc_;
    double max_cur_;
    double tread_;
    double front_suspension_;
    double rear_suspension_;
    double wheel_base_;
    double base_height_;
    double length_;
    double width_;
    double height_;



    Config(const ros::NodeHandle &nh_)
    {
        // nh_priv.getParam("MapTopic", mapTopic);
        // nh_priv.getParam("TargetTopic", targetTopic);
        // nh_priv.getParam("DilateRadius", dilateRadius);
        // nh_priv.getParam("VoxelWidth", voxelWidth);
        // nh_priv.getParam("MapBound", mapBound);
        // nh_priv.getParam("TimeoutRRT", timeoutRRT);
        // nh_priv.getParam("MaxVelMag", maxVelMag);
        // nh_priv.getParam("MaxBdrMag", maxBdrMag);
        // nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        // nh_priv.getParam("MinThrust", minThrust);
        // nh_priv.getParam("MaxThrust", maxThrust);
        // nh_priv.getParam("VehicleMass", vehicleMass);
        // nh_priv.getParam("GravAcc", gravAcc);
        // nh_priv.getParam("HorizDrag", horizDrag);
        // nh_priv.getParam("VertDrag", vertDrag);
        // nh_priv.getParam("ParasDrag", parasDrag);
        // nh_priv.getParam("SpeedEps", speedEps);
        // nh_priv.getParam("WeightT", weightT);
        // nh_priv.getParam("ChiVec", chiVec);
        // nh_priv.getParam("SmoothingEps", smoothingEps);
        // nh_priv.getParam("IntegralIntervs", integralIntervs);
        // nh_priv.getParam("RelCostTol", relCostTol);
      
      nh_.param<double>(ros::this_node::getName()+ "/height",height_,0.5);
      nh_.param<double>(ros::this_node::getName()+ "/length",length_,0.4);
      nh_.param<double>(ros::this_node::getName()+ "/width",width_,0.4);
      nh_.param<double>(ros::this_node::getName()+ "/max_vel",max_vel_,5);
      nh_.param<double>(ros::this_node::getName()+ "/max_acc",max_acc_,5);
      nh_.param<double>(ros::this_node::getName()+ "/max_cur",max_cur_,0.5);
      nh_.param<double>(ros::this_node::getName()+ "/tread",tread_,0.3);
      nh_.param<double>(ros::this_node::getName()+ "/front_suspension",front_suspension_,0.05);
      nh_.param<double>(ros::this_node::getName()+ "/rear_suspension",rear_suspension_,0.05);
      nh_.param<double>(ros::this_node::getName()+ "/wheel_base",wheel_base_,0.8);
      std::cout<<front_suspension_<<std::endl; ;
    }
};

class Gcopter
{
private:
  Config config_;
  ros::NodeHandle nh_;
  std::shared_ptr<World2oct> map_;
  std::shared_ptr<KinoAstar> kinoastar_;
  // std::shared_ptr<Visualizer> visualizer_;
  Trajectory<5, 2> traj;
  // minco::MINCO_S3NU minco;

  std::vector<std::vector<Eigen::MatrixX4d>> hPolyses;
  std::vector<std::vector<double>> hPolyTime;
  std::vector<std::vector<Eigen::Vector2d>> Innerpoint;
  std::vector<int> hPolynum;
  

  ros::Publisher meshPub;
  ros::Publisher edgePub;
  ros::Publisher box2DPub;
  ros::Publisher point_pub;

  double rho;
  double smoothEps;// smoothL1的范围
  double integralRes;// 积分分段数
  Eigen::VectorXd magnitudeBd;
  Eigen::VectorXd penaltyWt;

  double Freedom_;
  int piece_num_;
  std::vector<Eigen::MatrixXd> headPVA_;
  std::vector<Eigen::MatrixXd> tailPVA_;
  std::vector<int> singul_;
  // 或许需要删除内存？
  std::vector<minco::MINCO_S3NU> mincos;

public:


  inline bool setup(std::vector<FlatTrajData> &flat_trajs_){
    nh_.param<double>(ros::this_node::getName()+ "/TimeWeight", rho, 1);
    nh_.param<double>(ros::this_node::getName()+ "/smoothingFactor", smoothEps, 0.01);
    nh_.param<double>(ros::this_node::getName()+ "/integralResolution", integralRes, 12);

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

    piece_num_ = flat_trajs_.size();
    headPVA_.clear();
    tailPVA_.clear();
    singul_.clear();
    mincos.clear();
    for(int i=0;i<piece_num_;i++){
      Eigen::MatrixXd headPVA;
      headPVA.resize(Freedom_,3);
      Eigen::MatrixXd tailPVA;
      tailPVA.resize(Freedom_,3);
      headPVA = flat_trajs_[i].start_state;
      tailPVA = flat_trajs_[i].final_state;
      headPVA_.push_back(headPVA);
      tailPVA_.push_back(tailPVA);
      singul_.push_back(flat_trajs_[i].singul);
      hPolynum.push_back(hPolyses[i].size());
    
      minco::MINCO_S3NU minco;
      // minco.setConditions(headPVA,tailPVA)
      // 没写完！！！！！！！！！！！！！！！！！！！！！！！！！！！
    }



  }


  Gcopter(const Config &conf, ros::NodeHandle &nh, std::shared_ptr<World2oct> map, std::shared_ptr<KinoAstar> kinoastar):config_(conf){
    Freedom_ = 2;
    nh_ = nh;
    map_ = map;
    kinoastar_ = kinoastar;
    meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
    edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
    box2DPub = nh.advertise<visualization_msgs::Marker>("/visualizer/box2D", 1000);
    point_pub = nh_.advertise<visualization_msgs::MarkerArray>("/visualizer/innerpoint",100);
  }



  inline void plan(const std::vector<FlatTrajData> &flat_trajs_){
    get_hPolys(flat_trajs_);



  }

  // 获得飞行走廊
  inline void get_hPolys(const std::vector<FlatTrajData> &flat_trajs_){
    for(int i=0;i<flat_trajs_.size();i++){
      std::vector<Eigen::MatrixX4d> hPolys;
      std::vector<double> hPolystime;
      std::vector<Eigen::Vector2d> innerpoints;
      // std::vector<Eigen::VectorXd> route;
      // int flat_trajs_route_size = flat_trajs_[i].traj_pts.size();
      // for(int j=0;j<flat_trajs_route_size;j++){
      //   route.emplace_back(flat_trajs_[i].traj_pts[j].head(2));
      // }
      // convexCover_base(flat_trajs_[i].traj_pts,
      //                  map_->surf_,
      //                  Eigen::Vector3d(map_->x_lower,map_->y_lower,map_->z_lower),
      //                  Eigen::Vector3d(map_->x_upper,map_->y_upper,map_->z_upper),
      //                  2.0,
      //                  1.0,
      //                  hPolys);
      // // visual
      // vis_get_mesh(hPolys);
      // std::cout<<"start state: "<<flat_trajs_[i].start_state.col(0).transpose()<<std::endl;
      // std::cout<<"traj_pts:"<<std::endl;
      // for(int j=0;j<flat_trajs_[i].traj_pts.size();j++){
      //   std::cout<<flat_trajs_[i].traj_pts[j].transpose()<<std::endl;
      // }
      // std::cout<<"final state: "<<flat_trajs_[i].final_state.col(0).transpose()<<std::endl;

      // ROS_INFO("\033[41;37m  flat_trajs_[i].traj_pts.size():%d  theta.size():%d \033[0m",flat_trajs_[i].traj_pts.size(),flat_trajs_[i].thetas.size());  
      convexCover2D_base(flat_trajs_[i],
                         1.0,
                         hPolys);
      shortCut(hPolys);
      vis_2Dbox(i,hPolys);
      hPolyses.push_back(hPolys);

      getInnerPoints(flat_trajs_[i],hPolys,hPolystime,innerpoints);
      hPolyTime.push_back(hPolystime);
      Innerpoint.push_back(innerpoints);
      
      vis_innerPoint(i,innerpoints);
      std::cout<<"hPolystime:  ";
      for(int i=0;i<hPolystime.size();i++){
        std::cout<<hPolystime[i]<<"  ";
      }
      std::cout<<std::endl;
      ROS_INFO("\033[43;37m innerpoints.size:  %d, hPolys.size: %d \033[0m",innerpoints.size(),hPolys.size());    
    }
  }


//////////////////////////////////////////////////////////////////////////
  inline void convexCover_base(const std::vector<Eigen::Vector3d> &path,// 前端路径 
                               const std::vector<Eigen::Vector3d> &points,// surf实际坐标 
                               const Eigen::Vector3d &lowCorner,//  三维坐标最小值 
                               const Eigen::Vector3d &highCorner,//  三维坐标最大值
                               const double &progress,
                               const double &range,
                               std::vector<Eigen::MatrixX4d> &hpolys,
                               const double eps = 1.0e-6){
    hpolys.clear();
    const int n = path.size();
    Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
    bd(0, 0) = 1.0;
    bd(1, 0) = -1.0;
    bd(2, 1) = 1.0;
    bd(3, 1) = -1.0;
    bd(4, 2) = 1.0;
    bd(5, 2) = -1.0;
    Eigen::MatrixX4d hp, gap;
    Eigen::Vector3d a, b = Eigen::Vector3d(path[0].x(),path[0].y(),config_.height_/2);
    std::vector<Eigen::Vector3d> valid_pc;
    valid_pc.reserve(points.size());
    
    ros::Duration find_P_time;
    for (int i = 1; i < n;){
      
      a = b;
      if ((a.head(2) - path[i].head(2)).norm() > progress){
        b.head(2) = (path[i].head(2) - a.head(2)).normalized() * progress + a.head(2);
      }
      else{
        b.head(2) = path[i].head(2);
        i++;
      }
      // 组成一个立方体
      bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
      bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
      bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
      bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
      bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
      bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

      valid_pc.clear();
      for (const Eigen::Vector3d &p : points){
        // p在a b最大坐标+range 和 最小坐标-range内，则将这个点放进valid_pc
        if (p.z()<=config_.height_ && (bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0){
          valid_pc.emplace_back(p);
        }
      }

      
// ROS_INFO("\033[43;37m valid_pc.size(): %d \033[0m",valid_pc.size());
// ros::Time start = ros::Time::now();
      if(valid_pc.size()==0){
          hp.block<6, 4>(0, 0) = bd;
          hpolys.emplace_back(hp);
          continue;
      }
      // 将valid_pc变成一个3*n的矩阵pc
      Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());
      firi::firi(bd, pc, a, b, hp);

      if (hpolys.size() != 0){
          const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
          if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                        ((hpolys.back() * ah).array() > -eps).cast<int>().sum()){
              firi::firi(bd, pc, a, a, gap, 1);
              hpolys.emplace_back(gap);
          }
      }

      hpolys.emplace_back(hp);
// ros::Time end = ros::Time::now();
// find_P_time+=end-start;
  
    }
// ROS_INFO("\033[43;37m find valid_pc time: %f \033[0m",find_P_time.toSec());    
  }


//////////////////////////////////////////////////////////////////////////
  void vis_get_mesh(const std::vector<Eigen::MatrixX4d> &hPolys){
    Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);

    for (size_t id = 0; id < hPolys.size(); id++){

      oldTris = mesh;
      Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
      geo_utils::enumerateVs(hPolys[id], vPoly);
      quickhull::QuickHull<double> tinyQH;
      const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
      const auto &idxBuffer = polyHull.getIndexBuffer();
      int hNum = idxBuffer.size() / 3;
      curTris.resize(3, hNum * 3);
      for (int i = 0; i < hNum * 3; i++)
      {
          curTris.col(i) = vPoly.col(idxBuffer[i]);
      }
      mesh.resize(3, oldTris.cols() + curTris.cols());
      mesh.leftCols(oldTris.cols()) = oldTris;
      mesh.rightCols(curTris.cols()) = curTris;
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
    meshMarker.color.a = 0.15;
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;

    edgeMarker = meshMarker;
    edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 0.00;
    edgeMarker.color.g = 1.00;
    edgeMarker.color.b = 1.00;
    edgeMarker.color.a = 1.00;
    edgeMarker.scale.x = 0.02;

    geometry_msgs::Point point;

    int ptnum = mesh.cols();
// ROS_INFO("\033[43;37m visualization_msgs::Marker meshMarker, edgeMarker; ptnum: %d \033[0m",ptnum);
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
    edgePub.publish(edgeMarker);
    // ROS_INFO("\033[43;37m publish over! \033[0m");
  }

  // 处理一段固定方向的traj  //首末状态都给走廊   初末状态各一个，每一个innerpoint给一个
  void convexCover2D_base(const FlatTrajData &flat_traj,// 前端路径 
                          const double &range,
                          std::vector<Eigen::MatrixX4d> &hpolys,
                          const double eps = 1.0e-6){
    int traj_num = flat_traj.traj_pts.size();
    hpolys.clear();
    double resolution = map_->grid_interval_;
    // 走廊包含终点
    for(int i=0;i<traj_num;i++){
      Eigen::Matrix4d hPoly;
      
      // 前进速度
      double step = resolution;

      bool x_pos=true,y_pos=true,x_neg=true,y_neg=true;
      Eigen::Vector2d sourcePt = flat_traj.traj_pts[i].head(2);

      // 探测距离 x_pos,y_pos,x_neg,y_neg
      Eigen::Vector4d extend_length(config_.width_/2,
                                    config_.length_-config_.rear_suspension_,
                                    -config_.width_/2,
                                    -config_.rear_suspension_);
      // std::cout<<"extend_length"<<extend_length.transpose()<<std::endl;
      double yaw = flat_traj.thetas[i];
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
      //行向量储存 同gcopter一样
      hPoly(0,0) = sgn*(Corpoint2.y()-Corpoint1.y());
      hPoly(0,1) = sgn*(Corpoint1.x()-Corpoint2.x());
      hPoly(0,2) = 0;
      hPoly(0,3) = sgn*(Corpoint1.y()*Corpoint2.x() - Corpoint1.x()*Corpoint2.y());
      hPoly.row(0) = hPoly.row(0)/hPoly.row(0).head(3).norm();
      // 下边界
      dir = sourcePt.x()*(Corpoint3.y()-Corpoint2.y()) + sourcePt.y()*(Corpoint2.x()-Corpoint3.x())+Corpoint2.y()*Corpoint3.x()-Corpoint2.x()*Corpoint3.y();
      sgn = -dir/std::abs(dir);
      hPoly(1,0) = sgn*(Corpoint3.y()-Corpoint2.y());
      hPoly(1,1) = sgn*(Corpoint2.x()-Corpoint3.x());
      hPoly(1,2) = 0;
      hPoly(1,3) = sgn*(Corpoint2.y()*Corpoint3.x() - Corpoint2.x()*Corpoint3.y());
      hPoly.row(1) = hPoly.row(1)/hPoly.row(1).head(3).norm();
      // 左边界
      dir = sourcePt.x()*(Corpoint4.y()-Corpoint3.y()) + sourcePt.y()*(Corpoint3.x()-Corpoint4.x())+Corpoint3.y()*Corpoint4.x()-Corpoint3.x()*Corpoint4.y();
      sgn = -dir/std::abs(dir);
      hPoly(2,0) = sgn*(Corpoint4.y()-Corpoint3.y());
      hPoly(2,1) = sgn*(Corpoint3.x()-Corpoint4.x());
      hPoly(2,2) = 0;
      hPoly(2,3) = sgn*(Corpoint3.y()*Corpoint4.x() - Corpoint3.x()*Corpoint4.y());
      hPoly.row(2) = hPoly.row(2)/hPoly.row(2).head(3).norm();
      // 上边界
      dir = sourcePt.x()*(Corpoint1.y()-Corpoint4.y()) + sourcePt.y()*(Corpoint4.x()-Corpoint1.x())+Corpoint4.y()*Corpoint1.x()-Corpoint4.x()*Corpoint1.y();
      sgn = -dir/std::abs(dir);
      hPoly(3,0) = sgn*(Corpoint1.y()-Corpoint4.y());
      hPoly(3,1) = sgn*(Corpoint4.x()-Corpoint1.x());
      hPoly(3,2) = 0;
      hPoly(3,3) = sgn*(Corpoint4.y()*Corpoint1.x() - Corpoint4.x()*Corpoint1.y());
      if(std::isnan(hPoly(3,3))){
        std::cout<<"nan!!!"<<std::endl;
        std::cout<<sgn<<std::endl;
        std::cout<<dir<<std::endl;
        std::cout<<sourcePt.transpose()<<std::endl;
        std::cout<<extend_length.transpose()<<std::endl;
        std::cout<<Corpoint4.transpose()<<std::endl;
        std::cout<<Corpoint1.transpose()<<std::endl;
        std::cout<<hPoly<<std::endl;
      }
      hPoly.row(3) = hPoly.row(3)/hPoly.row(3).head(3).norm();
      if(std::isnan(hPoly(3,3))){
        std::cout<<"nan!!! hPoly.row(3)"<<std::endl;
        std::cout<<hPoly<<std::endl;
      }
      hpolys.push_back(hPoly);

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

  void vis_2Dbox(const int &i, const std::vector<Eigen::MatrixX4d> &hPoly){
    visualization_msgs::Marker rec;

    rec.type = visualization_msgs::Marker::LINE_LIST;
    rec.pose.orientation.w = 1.00;
    rec.ns = "2Dbox";
    rec.color.r = rand() / double(RAND_MAX);
    rec.color.g = rand() / double(RAND_MAX);
    rec.color.b = rand() / double(RAND_MAX);
    rec.color.a = 0.70;
    rec.scale.x = 0.01;
    rec.header.frame_id = "base";
    rec.lifetime = ros::Duration();
    rec.header.stamp = ros::Time::now();
    if (i == 0){
      rec.action = visualization_msgs::Marker::DELETEALL;
      box2DPub.publish(rec);
    }
    rec.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point point1;
    geometry_msgs::Point point2;
    geometry_msgs::Point point3;
    geometry_msgs::Point point4;
    point1.z = 0;
    point2.z = 0;
    point3.z = 0;
    point4.z = 0;
    for(int j=0;j<hPoly.size();j++){
      rec.id = j*10+i;
      point1.x =  (hPoly[j](1,3)*hPoly[j](0,1) - hPoly[j](0,3)*hPoly[j](1,1)) / (hPoly[j](0,0)*hPoly[j](1,1) - hPoly[j](1,0)*hPoly[j](0,1));
      point1.y =  (hPoly[j](0,3)*hPoly[j](1,0) - hPoly[j](1,3)*hPoly[j](0,0)) / (hPoly[j](0,0)*hPoly[j](1,1) - hPoly[j](1,0)*hPoly[j](0,1));
      point2.x =  (hPoly[j](2,3)*hPoly[j](1,1) - hPoly[j](1,3)*hPoly[j](2,1)) / (hPoly[j](1,0)*hPoly[j](2,1) - hPoly[j](2,0)*hPoly[j](1,1));
      point2.y =  (hPoly[j](1,3)*hPoly[j](2,0) - hPoly[j](2,3)*hPoly[j](1,0)) / (hPoly[j](1,0)*hPoly[j](2,1) - hPoly[j](2,0)*hPoly[j](1,1));
      point3.x =  (hPoly[j](3,3)*hPoly[j](2,1) - hPoly[j](2,3)*hPoly[j](3,1)) / (hPoly[j](2,0)*hPoly[j](3,1) - hPoly[j](3,0)*hPoly[j](2,1));
      point3.y =  (hPoly[j](2,3)*hPoly[j](3,0) - hPoly[j](3,3)*hPoly[j](2,0)) / (hPoly[j](2,0)*hPoly[j](3,1) - hPoly[j](3,0)*hPoly[j](2,1));
      point4.x =  (hPoly[j](0,3)*hPoly[j](3,1) - hPoly[j](3,3)*hPoly[j](0,1)) / (hPoly[j](3,0)*hPoly[j](0,1) - hPoly[j](0,0)*hPoly[j](3,1));
      point4.y =  (hPoly[j](3,3)*hPoly[j](0,0) - hPoly[j](0,3)*hPoly[j](3,0)) / (hPoly[j](3,0)*hPoly[j](0,1) - hPoly[j](0,0)*hPoly[j](3,1));
      // if(std::isnan(point1.x)){
      //   std::cout<<"nan!!!"<<std::endl;
      //   std::cout<<hPoly[j]<<std::endl;
      // }
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


  inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys){
    return;
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
            ROS_ERROR("htemp.size() is 1, Special treatment is still necessary here!!!");
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        Eigen::Vector4d center_p;
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 1; j < i; j++)
            {
                if (j < i - 1)
                {   
                    // overlap = geo_utils::overlap(htemp[i], htemp[j], center_p, std::max(config_.length_,config_.width_)/2);
                    overlap = geo_utils::overlap(htemp[i], htemp[j], center_p, config_.length_-config_.rear_suspension_);
                }
                else
                {
                    overlap = true;
                    // geo_utils::overlap(htemp[i], htemp[j], center_p);
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
        // hpolys.push_back(htemp[0]);
  }

  // 从已知路径中找一个点
  void getInnerPoints(const FlatTrajData &flat_traj,
                      const std::vector<Eigen::MatrixX4d> &hPolys,
                      std::vector<double> &hPolystime,
                      std::vector<Eigen::Vector2d> &innerpoints){


  for(int k=0;k<flat_traj.traj_pts.size();k++){
    std::cout<<flat_traj.traj_pts[k].transpose()<<"   ";
  }

    innerpoints.clear();
    hPolystime.clear();
    
    int traj_size = flat_traj.traj_pts.size()-1;
    int hPolys_size = hPolys.size();

    int index = 1;
    for(int j=0;j<hPolys_size-1;j++){
      double min_dis = -1e10;
      int min_index = -1;

      const int m = hPolys[j].rows();
      const int n = hPolys[j+1].rows();
      Eigen::MatrixX4d A(m + n, 4);
      A.topRows(m) = hPolys[j];
      A.bottomRows(n) = hPolys[j+1];

      for(int i=index;i<traj_size-1;i++){
        Eigen::Vector4d curpos(flat_traj.traj_pts[i].x(),flat_traj.traj_pts[i].y(),0,1);
        double cur_dis = -(A*curpos).maxCoeff();
        // std::cout<<"   cur_dis:"<<cur_dis;
        if(cur_dis > min_dis){
          min_dis = cur_dis;
          min_index = i;
        }
      }

      if(min_dis < 0){
        ROS_WARN("min_dis < 0! no point in both two hPolys !!!");
      }
      // ROS_INFO("min_index: %d",min_index);
      innerpoints.push_back(flat_traj.traj_pts[min_index].head(2));
      
      double total_time = 0;
      for(int i=1;i<=min_index;i++){
        total_time+=flat_traj.traj_pts[i].z();
      }
      hPolystime.push_back(total_time);
      index = min_index+1;
    }

  }

// 最小化航路点距离
  void getInnerPoints(const FlatTrajData &flat_traj,
                      const std::vector<Eigen::MatrixX4d> &hPolys,
                      const double &smoothD,
                      std::vector<Eigen::Vector2d> &innerpoints){
    int overlap = hPolys.size()-1;
    Eigen::VectorXd x(overlap*2);
    Eigen::Vector2d ini = flat_traj.start_state.col(0).head(2);
    Eigen::Vector2d fin = flat_traj.final_state.col(0).head(2);
    std::vector<Eigen::Vector2d> innerpointsw;
    std::vector<double> hPolystime;
  }

  void vis_innerPoint(const int &i, const std::vector<Eigen::Vector2d> &innerpoints){
    // ROS_INFO("\033[41;37m  innerpoints.size(): %d  \033[0m",innerpoints.size());  
    visualization_msgs::MarkerArray markerarraydelete;
    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "base";
    marker.ns = "innerPoint";
    marker.lifetime = ros::Duration();
    marker.type = visualization_msgs::Marker::CYLINDER;
    if(i==0){
      marker.action = visualization_msgs::Marker::DELETEALL;
      markerarraydelete.markers.push_back(marker);
      point_pub.publish(markerarraydelete);
    }
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.12;
    marker.scale.y = 0.12;
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

    std::cout<<"innerpoints[i].transpose()"<<std::endl;
    for(int j=0;j<innerpoints.size();j++){
      std::cout<<innerpoints[j].transpose()<<std::endl;
      marker.header.stamp = ros::Time::now();
      marker.id = j*100+i;
      marker.pose.position.x = innerpoints[j].x();
      marker.pose.position.y = innerpoints[j].y();
      std::cout<<marker.pose.position.x<<" "<<marker.pose.position.y<<std::endl;
      markerarray.markers.push_back(marker);
    }
    point_pub.publish(markerarray);
  }


};