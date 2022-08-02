#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "octomap_msgs/Octomap.h"
#include "octomap_msgs/conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "tf/transform_datatypes.h"


#include "front_end/kino_astar.h"
#include "gcopter/trajectory.hpp"

// debug
#include <fstream>

class Visualizer
{
  private:
    ros::NodeHandle nh_;

    ros::Publisher kinoastarFlatPathPubMarker;
    ros::Publisher kinoastarPubPCL;
    ros::Publisher finalnodePubMarker;
    ros::Publisher meshPub;
    ros::Publisher edgePub;
    ros::Publisher mincoCarPubMarker;
    ros::Publisher mincoPathPath;

    ros::Publisher debugCurPos;
  public:
    Visualizer(ros::NodeHandle nh){
      nh_ = nh;
      kinoastarFlatPathPubMarker = nh_.advertise<visualization_msgs::MarkerArray>("/visualizer/kinoastarFlatPath",10);
      kinoastarPubPCL = nh_.advertise<sensor_msgs::PointCloud2>("/visualizer/kinoastarPath",10);
      finalnodePubMarker = nh_.advertise<visualization_msgs::Marker>("/visualizer/finalnode",10);
      mincoCarPubMarker = nh_.advertise<visualization_msgs::Marker>("/visualizer/mincoCar",10);
      mincoPathPath = nh_.advertise<nav_msgs::Path>("/visualizer/mincoPath",10);
      
      debugCurPos = nh_.advertise<geometry_msgs::PoseStamped>("/debug/cur",10);
    }

        // routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        // wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        // trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
        // meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
        // edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);
        // spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);
        // speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
        // thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
        // tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
        // bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);
    ~Visualizer(){};

    void kinoastarPathPub(const std::vector<PathNodePtr> path){
      sensor_msgs::PointCloud2 globalMap_pcd;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr  colored_pcl_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZ> cloudMap;
      pcl::PointXYZRGB  pt;
      for(int i=0;i<path.size();i++){
        pt.x = path[i]->state.x();
        pt.y = path[i]->state.y();
        pt.z = 0;
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        colored_pcl_ptr->points.push_back(pt); 
      }
      cloudMap.height = colored_pcl_ptr->points.size();
      cloudMap.width = 1;
      // cloudMap.is_dense = true;
      pcl::toROSMsg(*colored_pcl_ptr,globalMap_pcd);
      globalMap_pcd.header.stamp = ros::Time::now();
      globalMap_pcd.header.frame_id = "base";
      kinoastarPubPCL.publish(globalMap_pcd);
    }

    void kinoastarFlatPathPub(const std::vector<FlatTrajData> flat_trajs){
      ROS_INFO("flat_trajs.size():  %d ",flat_trajs.size());
      for(int i=0;i<flat_trajs.size();i++){
        ROS_INFO("flat_trajs[%d].size():  %d ",i,flat_trajs[i].traj_pts.size());
      }  
 
      visualization_msgs::MarkerArray markerarraydelete;
      visualization_msgs::MarkerArray markerarray;
      visualization_msgs::Marker marker;
      // int marker_num = 100;
      marker.header.frame_id = "base";
      marker.ns = "kinoastarFlatPath";
      // marker.id = 0;
      //  marker.action = visualization_msgs::Marker::DELETEALL
      
      marker.lifetime = ros::Duration();
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::DELETEALL;
      marker.scale.x = 0.04;
      marker.scale.y = 0.04;
      marker.scale.z = 0.02;
      marker.color.a = 0.6;
      marker.color.r = 195.0/255;
      marker.color.g = 176.0/255;
      marker.color.b = 145.0/255;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;

      marker.header.stamp = ros::Time::now();
      marker.id = 0;
      marker.pose.position.x = flat_trajs[0].traj_pts[0].x();
      marker.pose.position.y = flat_trajs[0].traj_pts[0].y();
      markerarraydelete.markers.push_back(marker);
      kinoastarFlatPathPubMarker.publish(markerarraydelete);

      marker.action = visualization_msgs::Marker::ADD;
      for(int i=0;i<flat_trajs.size();i++){
        for(int j=0;j<flat_trajs[i].traj_pts.size();j++){
          marker.header.stamp = ros::Time::now();
          marker.id = j*100+i;
          marker.pose.position.x = flat_trajs[i].traj_pts[j].x();
          marker.pose.position.y = flat_trajs[i].traj_pts[j].y();
          markerarray.markers.push_back(marker);
        }
      }
      kinoastarFlatPathPubMarker.publish(markerarray);
    }

    void finalnodePub(const geometry_msgs::PoseStamped::ConstPtr &msg){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base";
      marker.ns = "finalnode";
      marker.lifetime = ros::Duration();
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.3;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 0.3;
      marker.color.r = rand() / double(RAND_MAX);
      marker.color.g = rand() / double(RAND_MAX);
      marker.color.b = rand() / double(RAND_MAX);
      marker.header.stamp = ros::Time::now();
      marker.id = 0;
      marker.pose.position.x = msg->pose.position.x;
      marker.pose.position.y = msg->pose.position.y;
      marker.pose.position.z = 0.25;
      marker.pose.orientation.w = msg->pose.orientation.w;
      marker.pose.orientation.x = msg->pose.orientation.x;
      marker.pose.orientation.y = msg->pose.orientation.y;
      marker.pose.orientation.z = msg->pose.orientation.z;
      finalnodePubMarker.publish(marker);
    }

    // void mincoCarPub(const Eigen::VectorXd &final_time, const std::vector<Trajectory<5, 2>> &final_trajes, const Eigen::VectorXi &final_singuls){
    void mincoCarPub(const std::vector<Trajectory<5, 2>> &final_trajes, const Eigen::VectorXi &final_singuls, const double &offsetx){
      if(final_trajes.size()!=final_singuls.size()){
        ROS_ERROR("[mincoCarPub] Input size ERROR !!!!");
      }
      int traj_size = final_trajes.size();
      double total_time;
      Eigen::VectorXd traj_time;
      traj_time.resize(traj_size);
      for(int i=0; i<traj_size; i++){
        traj_time[i] = final_trajes[i].getTotalDuration();
      }
      total_time = traj_time.sum();

      int index = 0;
      Eigen::VectorXd currPos, currVel, currAcc, currJer, currSna;
      Eigen::Vector2d offset;
      double current;
      offset << offsetx, 0;

      // 路径
      nav_msgs::Path path;
      path.header.frame_id = "base";
      path.header.stamp = ros::Time::now();
      
      for(double time = 1e-5; time<total_time; time+=1e-4){
        double index_time = 0;
        for(index = 0; index<traj_size; index++){
          if(time > index_time && time < index_time + traj_time[index] )
            break;
          index_time += traj_time[index];
        }
        currPos = final_trajes[index].getPos(time-index_time);
        currVel = final_trajes[index].getVel(time-index_time);
        currAcc = final_trajes[index].getAcc(time-index_time);
        currJer = final_trajes[index].getJer(time-index_time);
        currSna = final_trajes[index].getSna(time-index_time);
        // currJer = final_trajes[index].getJer(time-index_time);
        // std::cout<<"currPos: "<<currPos.transpose()<<"  currVel"<<currVel.transpose()<<std::endl;
        // std::cout<<"currPos: "<<currPos.transpose()<<"  currVel"<<currVel.transpose()<<"   currAcc: "<<currAcc.transpose()<<"  currJer"<<currJer.transpose()<<std::endl;
        // if(currVel.norm() < 1e-2)
        //   currVel = currVel/currVel.norm() * 1e-2;

        double yaw = atan2(currVel.y(),currVel.x());
        int singuls = final_singuls[index];
        if(singuls<0){
          yaw += M_PI;
        }
        Eigen::Matrix2d R;
        R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);

        
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = (currPos).x();
        pose.pose.position.y = (currPos).y();
        pose.pose.position.z = 0.15;
        
        pose.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
        path.poses.push_back(pose);
        
    // Eigen::Matrix2d B_h;
    // B_h << 0, -1.0,
    //        1.0,  0;
    // std::FILE *fp = fopen("/home/zmk/yunjing/220510Overall/cur.txt","a+");
    // double normVel = currVel.norm();
    // double help1 = 1/(normVel*normVel);
    // double help1_e = 1/(normVel*normVel+1e-8);
    // double omega =  help1* currAcc.transpose() * B_h * currVel;
    // double z_h0 = currVel.norm();
    // double z_h1 = currAcc.transpose() * currVel;
    // double z_h2 = currJer.transpose() * currVel;
    // double z_h3 = currAcc.transpose() * B_h * currVel;
    // double z_h4 = currJer.transpose() * B_h * currVel;
    // double z_h5 = currJer.transpose() * B_h * currAcc;
    // double z_h6 = currSna.transpose() * B_h * currVel;

    // double help2 = currJer.transpose()*B_h*currVel;
    // double domega = help2*help1 - 2.0 * help1*help1*currAcc.transpose()*B_h*currVel*currAcc.transpose()*currVel;
    // double domega2 = help2*help1 - 2.0 *help1*help1*z_h3*z_h1;
    // double domega_e = help2*help1_e - 2.0 * help1_e*help1_e*currAcc.transpose()*B_h*currVel*currAcc.transpose()*currVel;
    // double ddomega = (z_h5 + z_h6) * help1 + 
    //                  -(currAcc.squaredNorm()*z_h3 + z_h2*z_h3 + z_h4*2.0*z_h1) * 2.0  * help1 * help1 + 
    //                  8.0 * z_h3 * z_h1 * z_h1 * help1 * help1 * help1;
    // double cur = z_h3 * z_h3 * (help1 * help1 * help1);
    // double Accl = sqrt(z_h1 * z_h1 * help1);
    // fprintf(fp,"%.12f   ",time);
    // fprintf(fp,"%.12f   ",z_h0);
    // fprintf(fp,"%.12f   ",Accl);
    // fprintf(fp,"%.12f   ",cur);
    // fprintf(fp,"%.12f   ",omega);
    // fprintf(fp,"%.12f   ",domega);
    // fprintf(fp,"%.12f   ",domega_e);
    // fprintf(fp,"%.12f   \n",ddomega);
    // fclose(fp);

    }
      mincoPathPath.publish(path);



      // marker
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base";
      marker.ns = "mincoCar";
      marker.lifetime = ros::Duration();
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.a = 0.8;
      marker.color.r = rand() / double(RAND_MAX);
      marker.color.g = rand() / double(RAND_MAX);
      marker.color.b = rand() / double(RAND_MAX);
      marker.header.stamp = ros::Time::now();

      ros::Time start = ros::Time::now();
      for(current = (ros::Time::now()-start).toSec()/2; current<total_time; current = (ros::Time::now()-start).toSec()/2){

        marker.header.stamp = ros::Time::now();
        double index_time = 0;
        for( index = 0; index<traj_size; index++){
          if(current > index_time && current < index_time + traj_time[index] )
            break;
          index_time += traj_time[index];
        }
        currPos = final_trajes[index].getPos(current-index_time);
        currVel = final_trajes[index].getVel(current-index_time);
        
        // if(currVel.norm() < 1e-2)
        //   currVel = currVel/currVel.norm() * 1e-2;

        double yaw = atan2(currVel.y(),currVel.x());
        int singuls = final_singuls[index];
        if(singuls<0){
          yaw += M_PI;
        }
        Eigen::Matrix2d R;
        R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);

        // std::cout<<yaw<<"             "<<currVel.transpose()<<std::endl;


        // 车体
        offset << offsetx, 0;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.2;
        marker.id = 0;
        marker.pose.position.x = (R*offset).x() + currPos.x();
        marker.pose.position.y = (R*offset).y() + currPos.y();
        marker.pose.position.z = 0.2;
        marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
        mincoCarPubMarker.publish(marker);

        offset << 0, 0.15;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.id = 3;
        marker.pose.position.x = (R*offset).x() + currPos.x();
        marker.pose.position.y = (R*offset).y() + currPos.y();
        marker.pose.position.z = 0.05;
        marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
        mincoCarPubMarker.publish(marker);

        offset << 0, -0.15;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.id = 4;
        marker.pose.position.x = (R*offset).x() + currPos.x();
        marker.pose.position.y = (R*offset).y() + currPos.y();
        marker.pose.position.z = 0.05;
        marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
        mincoCarPubMarker.publish(marker);

        // 机臂
        offset << offsetx, 0.125;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.15;
        marker.id = 1;
        marker.pose.position.x = (R*offset).x() + currPos.x();
        marker.pose.position.y = (R*offset).y() + currPos.y();
        marker.pose.position.z = 0.375;
        marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
        mincoCarPubMarker.publish(marker);
        
        offset << offsetx, -0.125;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.15;
        marker.id = 2;
        marker.pose.position.x = (R*offset).x() + currPos.x();
        marker.pose.position.y = (R*offset).y() + currPos.y();
        marker.pose.position.z = 0.375;
        marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
        mincoCarPubMarker.publish(marker);
      }


    }

    void pub_singul_car(const Eigen::Vector4d &position){
      double offsetx = 0.15;
      double yaw = position[2];
      Eigen::Matrix2d R;
      R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);
      Eigen::Vector2d currPos(position[0], position[1]);
      
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base";
      marker.ns = "mincoCar";
      marker.lifetime = ros::Duration();
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.a = 1;
      marker.color.r = 0.3;
      marker.color.g = 0.75;
      marker.color.b = 0.82;
      marker.header.stamp = ros::Time::now();
      // 车体
      Eigen::Vector2d offset;

      offset << offsetx, 0;
      marker.scale.x = 0.4;
      marker.scale.y = 0.4;
      marker.scale.z = 0.2;
      marker.id = 0;
      marker.pose.position.x = (R*offset).x() + currPos.x();
      marker.pose.position.y = (R*offset).y() + currPos.y();
      marker.pose.position.z = 0.2;
      marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
      mincoCarPubMarker.publish(marker);
      
      
      offset << 0, 0.15;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.id = 3;
      marker.pose.position.x = (R*offset).x() + currPos.x();
      marker.pose.position.y = (R*offset).y() + currPos.y();
      marker.pose.position.z = 0.05;
      marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
      mincoCarPubMarker.publish(marker);

      offset << 0, -0.15;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.id = 4;
      marker.pose.position.x = (R*offset).x() + currPos.x();
      marker.pose.position.y = (R*offset).y() + currPos.y();
      marker.pose.position.z = 0.05;
      marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
      mincoCarPubMarker.publish(marker);

      // 机臂
      offset << offsetx, 0.125;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.15;
      marker.id = 1;
      marker.pose.position.x = (R*offset).x() + currPos.x();
      marker.pose.position.y = (R*offset).y() + currPos.y();
      marker.pose.position.z = 0.375;
      marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
      mincoCarPubMarker.publish(marker);
      
      offset << offsetx, -0.125;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.15;
      marker.id = 2;
      marker.pose.position.x = (R*offset).x() + currPos.x();
      marker.pose.position.y = (R*offset).y() + currPos.y();
      marker.pose.position.z = 0.375;
      marker.pose.orientation =tf::createQuaternionMsgFromYaw(yaw);
      mincoCarPubMarker.publish(marker);
    }



};



#endif