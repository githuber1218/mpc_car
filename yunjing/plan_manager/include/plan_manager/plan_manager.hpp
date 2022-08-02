#ifndef _PLAN_MANAGER_HPP_
#define _PLAN_MANAGER_HPP_

#include "model_gazebo/world2oct.h"
#include "visualizer/visualizer.hpp"
#include "front_end/kino_astar.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "back_end/gcopter.hpp"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"

class PlanManager
{
  private:
    ros::NodeHandle nh_;

    std::shared_ptr<World2oct> world2oct_;
    std::shared_ptr<Visualizer> visualizer_;
    std::shared_ptr<KinoAstar> kinoastar_;
    std::shared_ptr<Gcopter> gcopter_;


    ros::Subscriber goal_sub_;
    ros::Subscriber position_sub_;
    // ros::Subscriber current_state_sub_;

    ros::Publisher cmd_pub_;

    Eigen::Vector4d current_state_;
    Eigen::Vector2d current_control_;
    Eigen::Vector4d goal_state_;

    double dis_thre_;

    double max_vel_;
    double max_omega_;


  public:
    PlanManager(ros::NodeHandle nh){
      nh_ = nh;
      
      world2oct_ = std::make_shared<World2oct>(nh);
      visualizer_ = std::make_shared<Visualizer>(nh);
      kinoastar_ = std::make_shared<KinoAstar>(nh,world2oct_);
      gcopter_ = std::make_shared<Gcopter>(Config(ros::NodeHandle("~")), nh_, world2oct_,kinoastar_);
      ROS_INFO("kinoastar_->init();!!!!!!!!!!!!!");
      kinoastar_->init();

      goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",1,&PlanManager::goal_callback,this);
      position_sub_ = nh_.subscribe<nav_msgs::Odometry>("/vrpn_client_node/yunjing/pose",1,&PlanManager::pose_callback,this);
      cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

      nh_.param<double>(ros::this_node::getName()+ "/max_vel",max_vel_,5);
      nh_.param<double>(ros::this_node::getName()+ "/max_omega",max_omega_,5);

      nh_.param<double>(ros::this_node::getName()+ "/dis_thre",dis_thre_,0.01);
    
      // current_state_<<-4,2.5,0,0;
      // current_control_<<0,0;
      current_state_<<5.0,5.0,0,0;
      current_control_<<0,0;
      ros::Duration l(1);
      l.sleep();
      visualizer_->pub_singul_car(current_state_);
      
      // current_state_<<-2.00710201263, 3.01263618469, -0.5,0;
      // current_control_<<0,0;
      double v,omega;
      PurePursuit(Eigen::Vector2d(1.5,1),Eigen::Vector2d(2,3.5),M_PI/4,0.1,v,omega);
      // PurePursuit(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const double &theta, const double &dt, double &v, double &omega)
      ROS_INFO("asdasdasdasdsad: v :%f  omega:%f",v,omega);
    }

    ~PlanManager(){
      world2oct_->~World2oct();
      visualizer_->~Visualizer();
      kinoastar_->~KinoAstar();
    }

    void init(){
      kinoastar_->init();
    }

    void pose_callback(const nav_msgs::Odometry::ConstPtr &msg){

      Eigen::Quaterniond abcd(0.998914718628, -0.00357297854498, 0.000152770749992, 0.0464386790991);
      Eigen::Quaterniond pose(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
      pose = abcd.inverse() * pose;
      geometry_msgs::Quaternion ori;
      ori.w = pose.w();
      ori.x = pose.x();
      ori.y = pose.y();
      ori.z = pose.z();
      double yaw = tf::getYaw(ori)+M_PI;
      // yaw = yaw+1.9480-3.1415926  ;
      // std::cout<<yaw<<std::endl;  
      current_state_<<msg->pose.pose.position.x, msg->pose.pose.position.y, yaw, 0;
      visualizer_->pub_singul_car(current_state_);
    }

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg){
      std::cout << "get goal!" << std::endl;
      visualizer_->finalnodePub(msg);
      goal_state_ <<  msg->pose.position.x, msg->pose.position.y,
                  tf::getYaw(msg->pose.orientation), 1.0e-2;
      // goal_state_ << 2.24416303635,1.28502774239, 3.1415926535, 0.01;
      std::cout<<goal_state_.transpose()<<std::endl;
      std::cout<<"end_pt: "<<goal_state_.transpose()<<std::endl;

      int count = 0;
      while((current_state_.head(2)-goal_state_.head(2)).norm()>dis_thre_){
        ROS_INFO("finding Astar Road!");
        findAstarRoad();
        count++;
        ros::Time current = ros::Time::now();
        gcopter_->minco_plan();
        ROS_INFO("\033[41;37m all of minco plan time:%f \033[0m", (ros::Time::now()-current).toSec());
        ROS_INFO("mincoCarPub start!");
        visualizer_->mincoCarPub(gcopter_->final_trajes, gcopter_->final_singuls, gcopter_->offsetx);
        pub_cmd_vel(gcopter_->final_trajes, gcopter_->final_singuls, gcopter_->offsetx);
        std::cout<<goal_state_.transpose()<<std::endl;
        if(count == 1){
          ROS_ERROR("count is 50, cannot find road, break!");
          break;
        }
      }

      // current_state_ = goal_state_;
    }

    void findAstarRoad(){
      kinoastar_->reset();
      ros::Time current = ros::Time::now();
      kinoastar_->search(goal_state_,current_state_,current_control_);
      ROS_INFO("kinoastar time:%lf", (ros::Time::now()-current).toSec());
      if(kinoastar_->has_path_){
        kinoastar_->getKinoNode();
        
        visualizer_->kinoastarPathPub(kinoastar_->path_nodes_);
        visualizer_->kinoastarFlatPathPub(kinoastar_->flat_trajs_);
      }
    }

    // void pub_cmd_vel(const std::vector<Trajectory<5, 2>> &final_trajes, const Eigen::VectorXi &final_singuls, const double &offsetx){
    //   geometry_msgs::Twist cmd_vel;
      
    //   if(final_trajes.size()!=final_singuls.size()){
    //     ROS_ERROR("[mincoCarPub] Input size ERROR !!!!");
    //   }
    //   int traj_size = final_trajes.size();
    //   double total_time;
    //   Eigen::VectorXd traj_time;
    //   traj_time.resize(traj_size);
    //   for(int i=0; i<traj_size; i++){
    //     traj_time[i] = final_trajes[i].getTotalDuration();
    //   }
    //   total_time = traj_time.sum();

    //   int index = 0;
    //   Eigen::VectorXd currPos, currVel, currAcc, currJer, currSna;
    //   Eigen::Vector2d offset;
    //   double current;
    //   offset << offsetx, 0;

    //   Eigen::Matrix2d B_h;
    //   B_h << 0,-1,1,0;
      
    //   cmd_vel.angular.x = 0;
    //   cmd_vel.angular.y = 0;
    //   cmd_vel.linear.z = 0;
    //   ros::Time start = ros::Time::now();
    //   for(current = (ros::Time::now()-start).toSec(); current<total_time; current = (ros::Time::now()-start).toSec()){
    //     double index_time = 0;
    //     for( index = 0; index<traj_size; index++){
    //       if(current > index_time && current < index_time + traj_time[index] )
    //         break;
    //       index_time += traj_time[index];
    //     }
    //     currPos = final_trajes[index].getPos(current-index_time);
    //     currVel = final_trajes[index].getVel(current-index_time);
    //     currAcc = final_trajes[index].getAcc(current-index_time);

    //     if(currVel.norm() < 1e-8)
    //       currVel = currVel/currVel.norm() * 1e-8;

    //     double yaw = atan2(currVel.y(),currVel.x());
    //     int singuls = final_singuls[index];
    //     if(singuls<0){
    //       yaw += M_PI;
    //     }
    //     Eigen::Matrix2d R;
    //     R<<cos(yaw),-sin(yaw),sin(yaw),cos(yaw);

    //     double omega = 1.0/currVel.squaredNorm()*currAcc.transpose()*B_h*currVel;

    //     cmd_vel.linear.x = currVel.norm()*final_singuls[index];
    //     cmd_vel.angular.z = omega;

    //     cmd_pub_.publish(cmd_vel);

    //   }
    //   cmd_vel.linear.x = 0;
    //   cmd_vel.linear.y = 0;
    //   cmd_vel.angular.z = 0;
    //   cmd_pub_.publish(cmd_vel);

    // }

    void pub_cmd_vel(const std::vector<Trajectory<5, 2>> &final_trajes, const Eigen::VectorXi &final_singuls, const double &offsetx){
      geometry_msgs::Twist cmd_vel;
      
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

      Eigen::VectorXd currPos, currVel, currAcc, currJer, currSna;
      double current;

      // 控制的前视距离
      double dt = 0.1;

      cmd_vel.angular.x = 0;
      cmd_vel.angular.y = 0;
      cmd_vel.linear.z = 0;
      cmd_vel.linear.y = 0;
      
      
      for(int traj_i = 0; traj_i<traj_size; traj_i++){
        ros::Time start = ros::Time::now();
        double piece_time = final_trajes[traj_i].getTotalDuration();
        for(current = (ros::Time::now()-start).toSec(); current<piece_time; current = (ros::Time::now()-start).toSec()){
          ros::spinOnce();
          
          double goal_time = current + dt;
          goal_time = goal_time < piece_time? goal_time : piece_time-1e-5;
          currPos = final_trajes[traj_i].getPos(goal_time);
          PurePursuit(current_state_.head(2), currPos, current_state_[2], dt, cmd_vel.linear.x, cmd_vel.angular.z);
          cmd_vel.linear.x = -final_singuls[traj_i] * cmd_vel.linear.x;
          // ROS_INFO("current_state_:%f  %f   %f  %f   ,cmd:%f    %f",current_state_.x(),current_state_.y(),current_state_.z(),current_state_.w(),cmd_vel.linear.x,cmd_vel.angular.z);
          cmd_pub_.publish(cmd_vel);

        }
      }

      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      cmd_pub_.publish(cmd_vel);

    }

    void PurePursuit(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const double &theta, const double &dt, double &v, double &omega){

      Eigen::Vector2d v1(cos(theta),sin(theta));
      // std::cout<<p1<<" "<<p2<<" "<<v1<<" "<<dt<<std::endl;
      Eigen::Matrix2d A;
      A.block(0,0,1,2) = (p2 - p1).transpose();;
      A.block(1,0,1,2) = v1.transpose();

      Eigen::Vector2d b(0.5 * (p1.squaredNorm() - p2.squaredNorm()), -p1.dot(v1));
      Eigen::Vector2d po = -A.inverse()*b;

      // std::cout<<A<<std::endl<<std::endl;
      // std::cout<<A.inverse()<<std::endl<<std::endl;
      // std::cout<<b<<std::endl<<std::endl;
      // std::cout<<p2<<std::endl<<std::endl;
      // std::cout<<p1<<std::endl<<std::endl;
      // std::cout<<po<<std::endl<<std::endl<<std::endl;
      // std::cout<<max_vel_<<std::endl<<std::endl<<std::endl;
      // ROS_INFO("%f   %f",(b-p1).norm(),(b-p2).norm());
      double dirc = (v1[0]*(po-p1)[1] - v1[1]*(po-p1)[0])>0? 1:-1;

      double R = (p1-po).norm();
      double alpha = asin((p2-p1).norm()/2.0/R)*dirc;
      // ROS_INFO("%f   %f",(p2-p1).norm(),R*alpha);
      v = 2*R*alpha/dt;
      omega = dirc * abs(2*alpha/dt);

      omega = dirc*std::min(abs(2*alpha/dt), max_omega_*2)*0.25;
      v = std::min(abs(2*R*alpha/dt), max_vel_*3)*(1-abs(omega)/max_omega_/2/0.3);
      // v = std::min(abs(2*R*alpha/dt), 100000.0);
      // omega = dirc*std::min(abs(2*alpha/dt), 10000.0);

    }
};


#endif