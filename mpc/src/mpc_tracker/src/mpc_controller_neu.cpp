#include <iostream>
#include "Eigen/Eigen"
#include "OsqpEigen/OsqpEigen.h"
#include "ros/ros.h"
#include "mpc_tracker/mpc_tracker.hpp"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

Eigen::VectorXd current_state(3);
nav_msgs::Odometry frame;


void pose_callback(const nav_msgs::Odometry::ConstPtr& msg){
  Eigen::Quaterniond pose(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);

  geometry_msgs::Quaternion ori;
  ori.w = pose.w();
  ori.x = pose.x();
  ori.y = pose.y();
  ori.z = pose.z();
  double yaw = tf::getYaw(ori)+M_PI/2;
  // std::cout<<yaw<<std::endl;  
  current_state<<msg->pose.pose.position.x, msg->pose.pose.position.y, yaw;//no z was transported

  frame = *msg;

  //the parameter in the function needs to bechange
  //is current_state_ a global variable?
  frame.header.frame_id = "map";
  frame.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
}

int main(int argc,char **argv){
  ros::init(argc,argv,"mpc_controller");
  ros::NodeHandle n;
  ros::Publisher cmd_pub_;
  ros::Publisher framer_;
  ros::Subscriber position_sub_;
  ros::Publisher ref_pub;
  int number = 501;
  double dt = 0.02;
  ref_pub = n.advertise<nav_msgs::Path>("ref_traj", 1);
  current_state << 2, 0, M_PI/2;

  ros::Time current_time,last_time;
  current_time = ros::Time::now();//can be used for timestamped
  last_time = ros::Time::now();

  mpc mpc_;

  //function to introduce the reference path
  double x_ref[number], y_ref[number], theta_ref[number];
  for(int i=0;i<number;i++){
    // x_ref[i] = i*0.02-5; 
    // y_ref[i] = 0;
    // theta_ref[i] = 0;
    x_ref[i] = 2*cos(2*M_PI/(number-1)*i);
    y_ref[i] = 2*sin(2*M_PI/(number-1)*i);
    theta_ref[i] = M_PI/2 + 2*M_PI/(number-1)*i;
  }
  Eigen::VectorXd *refstate = new Eigen::VectorXd[number];
  Eigen::VectorXd *refinput = new Eigen::VectorXd[number-1];

  for(int i=0;i<number-1;i++){
    refstate[i].resize(3);
    refinput[i].resize(2);
    refstate[i][0] = x_ref[i];
    refstate[i][1] = y_ref[i];
    // refstate[i][2] = atan2(y_ref[i+1]-y_ref[i],x_ref[i+1]-x_ref[i]);
    refstate[i][2] = theta_ref[i];
    refinput[i][0] = 0.2;
    refinput[i][1] = 0.5;
    // refinput[i][0] = 1;
    // refinput[i][1] = 0;
  }
  refstate[number-1].resize(3);
  refstate[number-1][0] = x_ref[number-1];
  refstate[number-1][1] = y_ref[number-1];
  refstate[number-1][2] = theta_ref[number-1];

  dt = 2*M_PI/(number-1)/0.5;

  nav_msgs::Path ref_traj;
  geometry_msgs::PoseStamped pose_stamped;
  current_time = ros::Time::now();
  

  ref_traj.header.stamp = current_time;
  ref_traj.header.frame_id = "map";

  for(int i=0;i<number;i++){
    pose_stamped.pose.position.x = refstate[i][0];
    pose_stamped.pose.position.y = refstate[i][1];
    pose_stamped.pose.position.z = 0;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(refstate[i][2]);
    pose_stamped.pose.orientation.x = goal_quat.x;
    pose_stamped.pose.orientation.y = goal_quat.y;
    pose_stamped.pose.orientation.z = goal_quat.z;
    pose_stamped.pose.orientation.w = goal_quat.w;

    pose_stamped.header.stamp = current_time;
    pose_stamped.header.frame_id = "map";
    ref_traj.poses.push_back(pose_stamped);
  }

  // the parameter can be changed
  Eigen::MatrixXd q,r;
  q.resize(3,3);
  r.resize(2,2);
  q << 10,0,0, 0,10,0, 0,0,10;
  r << 1,0, 0,1;
  mpc_.SetConfig(q,r,dt);

  Eigen::VectorXd state_min(3),state_max(3),u_min(2),u_max(2),delta_umin(2),delta_umax(2);
  state_min << -100000, -100000,-100000;
  state_max << 100000, 100000, 100000;
  u_min << -1, -2;
  u_max << 1, 2;
  delta_umin << -2, -2;
  delta_umax << 2, 2;
  mpc_.SetConstraints(state_min,state_max,u_min,u_max,delta_umin,delta_umax);
  //the constrains

  Eigen::VectorXd real_input;
  int idx = 0;

  ros::Rate rr(100);//to control the frequency
  cmd_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  position_sub_ = n.subscribe<nav_msgs::Odometry>("/vrpn_client_node/yunjing_ww/pose",1,pose_callback);//the name should be changed
  framer_ = n.advertise<nav_msgs::Odometry>("framer_",1);

  while(n.ok()){
    current_time = ros::Time::now();
    ref_traj.header.stamp = current_time;

    ref_pub.publish(ref_traj);
    geometry_msgs::Twist geo_msgs;//needs to send to the "cmd_vel"
    
    ros::spinOnce();//what if we use the r.sleep()
    //the meaning of nav_msg and the Pathmanager needs to be specified
    //don't sure if the nav_msgs and geometry_msgs are definied
    
    real_input = mpc_.getresult(current_state, idx ,refstate, refinput, number);
    geo_msgs.linear.x = real_input[0];
    geo_msgs.angular.z = real_input[1];
    
    framer_.publish(frame);
    cmd_pub_.publish(geo_msgs);
       
    // last_time = current_time;
    rr.sleep();
    }

 


  return 0;
}
