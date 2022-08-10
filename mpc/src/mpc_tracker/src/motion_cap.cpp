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



