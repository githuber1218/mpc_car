#include <iostream>
#include "mpc_tracker/mpc_tracker.hpp"



void update(Eigen::VectorXd &state_now, const Eigen::VectorXd &real_input)
{
  double dt = 0.01;
  state_now[0] += real_input[0]*cos(state_now[2])*dt;
  state_now[1] += real_input[0]*sin(state_now[2])*dt;
  state_now[2] += real_input[1]*dt;
}


int main(int argc,char **argv){
  ros::init(argc,argv,"mpc_tracker");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher ref_pub;
  ref_pub = n.advertise<nav_msgs::Path>("ref_traj", 1);

  double x_ref[1001];
  for(int i=0;i<1001;i++){
    x_ref[i] = i*0.02; 
  }
  Eigen::VectorXd *refstate = new Eigen::VectorXd[1001];
  Eigen::VectorXd *refinput = new Eigen::VectorXd[1000];
  for(int i=0;i<1000;i++){
    refstate[i].resize(3);
    refinput[i].resize(2);
    refstate[i][0] = x_ref[i];
    refstate[i][1] = 0;
    refstate[i][2] = 0;
    refinput[i][0] = 1;
    refinput[i][1] = 0;
  }
  refstate[1000].resize(3);
  refstate[1000][0] = x_ref[1000];
  refstate[1000][1] = 0;
  refstate[1000][2] = 0;

  nav_msgs::Path ref_traj;
  geometry_msgs::PoseStamped pose_stamped;
  ros::Time current_time = ros::Time::now();
  int number = 1001;

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



  Eigen::VectorXd state_now(3);
  state_now[0] = 0;
  state_now[1] = 0;
  state_now[2] = -M_PI/4;

  mpc mpc_controller;
  
  Eigen::MatrixXd q,r;
  q.resize(3,3);
  r.resize(2,2);
  q << 1,0,0, 0,5,0, 0,0,1;
  r << 1,0, 0,1;
  mpc_controller.SetConfig(q,r);

  Eigen::VectorXd state_min(3),state_max(3),u_min(2),u_max(2),delta_umin(2),delta_umax(2);
  state_min << -1, -1, -1.8;
  state_max << 5, 1, 1.8;
  u_min << -1, -1;
  u_max << 1, 1;
  delta_umin << -1, -1;
  delta_umax << 1, 1;
  mpc_controller.SetConstraints(state_min,state_max,u_min,u_max,delta_umin,delta_umax);

  Eigen::VectorXd real_input;
  int idx = 0;


  while(idx!=number-1){
    current_time = ros::Time::now();
    ref_traj.header.stamp = current_time;

    ref_pub.publish(ref_traj);
    real_input = mpc_controller.getresult(state_now, idx, refstate, refinput, number);
    update(state_now, real_input);
    loop_rate.sleep();
  }
  
  return 0;
}
