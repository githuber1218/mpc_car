#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

int main(int argc,char **argv){
    ros::init(argc,argv,"geo_try");
    ros::NodeHandle n;
    ros::Publisher cmd_pub_;
    geometry_msgs::Twist geo_try_msgs;
    cmd_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    geo_try_msgs.linear.x = 1;
    geo_try_msgs.linear.y = 0;
    geo_try_msgs.linear.z = 0;
    geo_try_msgs.angular.x = 0;
    geo_try_msgs.angular.y = 0;
    geo_try_msgs.angular.z = 0;
    ros::Rate r(24.0);
while(n.ok()){
    cmd_pub_.publish(geo_try_msgs);
    r.sleep();
    }
}
