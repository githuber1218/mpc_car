#include "plan_manager/plan_manager.hpp"
#include "visualizer/visualizer.hpp"



int main(int argc,char **argv){
  ros::init(argc, argv, "world2oct");
  ros::NodeHandle nh("~");
  PlanManager* planmanager = new PlanManager(nh);
  ros::spin();


  // std::cout<<ros::this_node::getName()<<std::endl;

  return 0;
}