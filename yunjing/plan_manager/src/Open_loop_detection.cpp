#include "geometry_msgs/Twist.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>



int main(int argc,char **argv){
  ros::init(argc, argv, "open_loop_detection");
  ros::NodeHandle nh("~");

  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
  geometry_msgs::Twist cmd_order;
  cmd_order.angular.x = 0;
  cmd_order.angular.y = 0;
  cmd_order.linear.y = 0;
  cmd_order.linear.z = 0;

  cmd_order.angular.z = 0;
  cmd_order.linear.x = 0;

  double domega = 1.0;
  double maxomega = 0.5;
  double finalyaw = 4.0*M_PI;

  double change_time = maxomega/domega;
  double all_time = 2*change_time + (finalyaw-maxomega*maxomega/domega)/maxomega;

  double yaw = 0.0;
  double old_current = 0;
  double old_omega = 0;
  
  ros::Time start = ros::Time::now();
  for(double current = (ros::Time::now()-start).toSec(); current<all_time; current = (ros::Time::now()-start).toSec()){
    old_omega = cmd_order.angular.z;
    if(current < change_time){
      cmd_order.angular.z = current*domega;
    }
    else if(current > all_time-change_time){
      cmd_order.angular.z = maxomega - (current - (all_time-change_time))*domega;
    }
    else
      cmd_order.angular.z = maxomega;


    cmd_pub.publish(cmd_order);
    yaw += (current-old_current)*old_omega;
    old_current = current;
    if(current-floor(current)<0.000005)
    ROS_INFO("current yaw: %f   old_omega:%f   current:%f",yaw,cmd_order.angular.z,current);
  }

  ROS_INFO("final yaw: %f",yaw);
  ros::spinOnce();


  // std::cout<<ros::this_node::getName()<<std::endl;

  return 0;
}