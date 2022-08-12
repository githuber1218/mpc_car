#include <iostream>
#include "mpc_tracker/mpc_tracker.hpp"



void update(Eigen::VectorXd &state_now, const Eigen::VectorXd &real_input, int &idx, const Eigen::VectorXd* refstate, const int &number)
{
  double dt = 0.01;

      //   int i;
      // double dist=0;
      // double dist_min=10000;
      // for(i=0;i<number;i++){
      //   dist = pow(refstate[i][0] - state_now[0], 2) + pow(refstate[i][1] - state_now[1], 2);
      //   if(dist<dist_min){
      //     dist_min = dist;
      //     idx = i;
      //     // std::cout << "dist_min:" << std::endl << dist_min << std::endl;
      //     // std::cout << "idx:" << std::endl << idx << std::endl;
      //   }
      // }

  double refx = refstate[idx][0];
  double refy = refstate[idx][1];
  double reftheta = refstate[idx][2];
  state_now[3] = pow(sin(reftheta)*state_now[0] - cos(reftheta)*state_now[1] + cos(reftheta)*refy - refx*sin(reftheta),2.0);
  state_now[0] += real_input[0]*cos(state_now[2])*dt;
  state_now[1] += real_input[0]*sin(state_now[2])*dt;
  state_now[2] += real_input[1]*dt;
  // state_now[3] = pow(sin(reftheta)*state_now[0] - cos(reftheta)*state_now[1] + cos(reftheta)*refy - refx*sin(reftheta),2.0);
}


int main(int argc,char **argv){
  ros::init(argc,argv,"mpc_tracker");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher ref_pub;
  ref_pub = n.advertise<nav_msgs::Path>("ref_traj", 1);

  int number = 1001;
  double x_ref[number];
  double y_ref[number];
  double theta_ref[number];
  for(int i=0;i<number;i++){
    // x_ref[i] = i*0.02-5;
    x_ref[i] = 2*cos(2*M_PI/(number-1)*i);
    y_ref[i] = 2*sin(2*M_PI/(number-1)*i);
    theta_ref[i] = M_PI/2 + 2*M_PI/(number-1)*i;
  }
  Eigen::VectorXd *refstate = new Eigen::VectorXd[number];
  Eigen::VectorXd *refinput = new Eigen::VectorXd[number - 1];
  for(int i=0;i<number - 1;i++){
    refstate[i].resize(4);
    refinput[i].resize(2);
    refstate[i][0] = x_ref[i];
    refstate[i][1] = y_ref[i];
    refstate[i][2] = theta_ref[i];
    refstate[i][3] = 0;
    refinput[i][0] = 1;
    refinput[i][1] = 0;
  }
  refstate[number - 1].resize(4);
  refstate[number - 1][0] = x_ref[number - 1];
  refstate[number - 1][1] = y_ref[number - 1];
  refstate[number - 1][2] = theta_ref[number - 1];
  refstate[number - 1][3] = 0;

  nav_msgs::Path ref_traj;
  geometry_msgs::PoseStamped pose_stamped;
  ros::Time current_time = ros::Time::now();
  

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



  Eigen::VectorXd state_now(4);
  state_now[0] = 0;
  state_now[1] = 0;
  state_now[2] = -M_PI/4;
  state_now[3] = 0;

  mpc mpc_controller;
  
  Eigen::MatrixXd q,r;
  q.resize(4,4);
  r.resize(2,2);
  q << 1,0,0,0 , 0,1,0,0 , 0,0,1,0, 0, 0, 0, 10;
  r << 1,0, 0,1;
  mpc_controller.SetConfig(q,r);

  Eigen::VectorXd state_min(4),state_max(4),u_min(2),u_max(2),delta_umin(2),delta_umax(2);
  state_min << -1000, -1000, -1000,-1000;
  state_max << 1000, 1000, 1000,1000;
  u_min << -1, -1;
  u_max << 1, 1;
  delta_umin << -1, -1;
  delta_umax << 1, 1;
  mpc_controller.SetConstraints(state_min,state_max,u_min,u_max,delta_umin,delta_umax);

  Eigen::VectorXd real_input;
  int idx = 0;


  while((ros::ok())&&(idx!=number-1)){
    current_time = ros::Time::now();
    ref_traj.header.stamp = current_time;

    ref_pub.publish(ref_traj);
    real_input = mpc_controller.getresult(state_now, idx, refstate, refinput, number);
    update(state_now, real_input,idx,refstate,number);
    std::cout << "state_now:" << std::endl << state_now << std::endl;
    loop_rate.sleep();
  }
  
  return 0;
}
