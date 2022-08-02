#include <istream>
#include <ros/ros.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/SetJointProperties.h>
#include <gazebo_msgs/ODEJointProperties.h>
#include <gazebo_msgs/SetModelConfiguration.h>


ros::ServiceClient client;
ros::ServiceClient client2;

double angle = 0.1;


void TimeCallback(const ros::TimerEvent&){
  gazebo_msgs::SetModelConfiguration modelConf;
  modelConf.request.model_name = "unit_box_small";
  modelConf.request.joint_names.push_back("joint_left");
  modelConf.request.joint_positions.push_back(angle+=0.01);
  modelConf.request.urdf_param_name = "";
  client2.call(modelConf);
}

int main(int argc, char** argv){
  ros::init(argc,argv,"msg_exam");
  ros::NodeHandle nh;
  client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  client2 = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
  // gazebo_msgs::SetModelState objstate;

  // objstate.request.model_state.model_name = "unit_box";
  // objstate.request.model_state.pose.position.x = 5;
  // objstate.request.model_state.pose.position.y = 5;
  // objstate.request.model_state.pose.position.z = 5;
  // objstate.request.model_state.pose.orientation.w = 1;
  // objstate.request.model_state.pose.orientation.x = 0;
  // objstate.request.model_state.pose.orientation.y = 0;
  // objstate.request.model_state.pose.orientation.z = 0;
  // objstate.request.model_state.twist.linear.x = 0.0;
  // objstate.request.model_state.twist.linear.y = 0.0;
  // objstate.request.model_state.twist.linear.z = 0.0;
  // objstate.request.model_state.twist.angular.x = 0.0;
  // objstate.request.model_state.twist.angular.y = 0.0;
  // objstate.request.model_state.twist.angular.z = 0.0;
  // objstate.request.model_state.reference_frame = "world";
  // client.call(objstate);

  


  
  ros::Timer Timer = nh.createTimer(ros::Duration(0.01), TimeCallback, false,true);



  ros::spin();
  ROS_INFO("Over!!");
  return 0;
}