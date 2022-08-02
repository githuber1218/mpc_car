#ifndef _WORLD2OCT_H_
#define _WORLD2OCT_H_

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
#include <sensor_msgs/PointCloud2.h>


#include <tinyxml2.h>

#define Unoccupied 0
#define Occupied 1

class World2oct
{
  private:
    
    std::shared_ptr<octomap::OcTree> octomap_;
    
    ros::NodeHandle nh_;
    ros::Publisher pub_octomap_;
    ros::Publisher pub_gridmap_;
    ros::Publisher pub_surf_;


    // ros::Timer timer;


  public:
    // octomap间隔
    double interval_;

    uint8_t *gridmap_;
    double grid_interval_;
    double inv_grid_interval_;
    double x_upper, y_upper, z_upper;
	  double x_lower, y_lower, z_lower;
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
	  int GLXYZ_SIZE, GLYZ_SIZE;

    std::vector<Eigen::Vector3i> surf_id_;
    std::vector<Eigen::Vector3d> surf_;

    World2oct(ros::NodeHandle nh){
      nh_ = nh;

      get_octomap_map_ = false;
      get_grid_map_ = false;

      std::string interval_path = ros::this_node::getName() + "/octomap_interval";
      std::string grid_interval_path = ros::this_node::getName() + "/octomap_interval";
      std::string file_name_path = ros::this_node::getName() + "/file_name";
      nh_.param<double>(interval_path,interval_,0.05);
      nh_.param<double>(ros::this_node::getName()+"/gridmap_interval",grid_interval_,0.1);
      inv_grid_interval_ = 1/grid_interval_;

      octomap_ = std::make_shared<octomap::OcTree>(interval_);
      octomap::point3d max(DBL_MAX,DBL_MAX,DBL_MAX);
      octomap::point3d min(-DBL_MAX,-DBL_MAX,-DBL_MAX);
      octomap_->setBBXMax(min);
      octomap_->setBBXMin(max);
      std::cout<<max<<min<<std::endl;
// ROS_INFO("\033[42;37m getBBXMax():%f %f %f getBBXMin():%f %f %f \033[0m",octomap_->getBBXMax().x(),octomap_->getBBXMax().y(),octomap_->getBBXMax().z(),octomap_->getBBXMin().x(),octomap_->getBBXMin().y(),octomap_->getBBXMin().z());
  
      pub_octomap_ = nh.advertise<octomap_msgs::Octomap>("/octomap",1);
      pub_gridmap_ = nh.advertise<sensor_msgs::PointCloud2>("/gridmap",1);
      pub_surf_ = nh.advertise<sensor_msgs::PointCloud2>("/surface",1);
      
      std::string file_name;
      nh.param<std::string>(file_name_path,file_name,"yunjing_world.world");
      // get_oct_from_xml(file_name.c_str());
      get_oct_from_yaml();
// ROS_INFO("\033[41;37m x_upper:%f x_lower:%f y_upper:%f y_lower:%f z_upper:%f z_lower:%f \033[0m",x_upper,x_lower,y_upper,y_lower,z_upper,z_lower);
// ROS_INFO("\033[42;37m getBBXMax():%f %f %f getBBXMin():%f %f %f \033[0m",octomap_->getBBXMax().x(),octomap_->getBBXMax().y(),octomap_->getBBXMax().z(),octomap_->getBBXMin().x(),octomap_->getBBXMin().y(),octomap_->getBBXMin().z());
      ROS_INFO("get_surf();");
      get_surf();
      ROS_INFO("get_surf(); over!");
      ros::Duration(2).sleep();

      publish_octmap();
      publish_surf();
      publish_gridmap();
    }
    ~World2oct(){
      delete[] gridmap_;
      gridmap_ = nullptr;
      octomap_->~OcTree();
    };


    bool get_octomap_map_;
    bool get_grid_map_;
    

    //get octomap from world
    inline void StringPose2Vector(std::string numStr,Eigen::Vector3f* location,Eigen::Vector3f* euler);
    inline void StringSize2Vector(std::string numStr,Eigen::Vector3f* size);
    bool get_oct_from_xml(std::string Path);
    inline void insertbox(Eigen::Vector3f location,Eigen::Matrix3f euler,Eigen::Vector3f size);

    bool get_oct_from_yaml();

    //for gridmap
    Eigen::Vector3f gridIndex2coord(const Eigen::Vector3i &index);
	  Eigen::Vector3i coord2gridIndex(const Eigen::Vector3f &pt);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d &pt);
    Eigen::Vector2i coord2grid2dIndex(const Eigen::Vector2d &xypos);
    void setObs(const Eigen::Vector3f coord);
    inline void grid_insertbox(Eigen::Vector3f location,Eigen::Matrix3f euler,Eigen::Vector3f size);
    Eigen::Vector3i vectornum2gridIndex(const int num);
    uint8_t CheckCollisionBycoord(const Eigen::Vector3f &pt);
    uint8_t CheckCollisionBycoord(const Eigen::Vector3d &pt);
    uint8_t CheckCollisionBycoord(const double ptx,const double pty,const double ptz);


    
    //get and set parameter
    double get_interval() {return interval_;}  
    void set_octomap(std::shared_ptr<octomap::OcTree> octomap) {octomap = octomap_;}
    
    // visualization
    void publish_octmap();
    void publish_gridmap();
    //exam function
    void TimeCallback(const ros::TimerEvent&);

    // get surface
    void get_surf();
    void publish_surf();

    void find_point_occupany(double x,double y,double z){
      octomap_->updateNode(octomap::point3d(1,1,1),true);
      octomap::OcTreeNode* result = octomap_->search(1,1,1);
      std::cout<<"result->getOccupancy():  "<<result->getOccupancy()<<std::endl;
      std::cout<<"result->getValue():  "<<result->getValue()<<std::endl;
      octomap_->updateNode(octomap::point3d(10,10,10),false);
      result = octomap_->search(10,10,10);
      std::cout<<"result->getOccupancy():  "<<result->getOccupancy()<<std::endl;
      std::cout<<"result->getValue():  "<<result->getValue()<<std::endl;
      result = octomap_->search(0,0,0);
      if(result==NULL){
        return;
      }
      std::cout<<"result->getOccupancy():  "<<result->getOccupancy()<<std::endl;
      std::cout<<"result->getValue():  "<<result->getValue()<<std::endl;
    }

};

#endif
