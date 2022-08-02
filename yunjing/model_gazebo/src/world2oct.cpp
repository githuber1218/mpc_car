#include "model_gazebo/world2oct.h"
// #include <eigen3/unsupported/Eigen/CXX11/Tensor>

inline void World2oct::StringPose2Vector(std::string numStr,Eigen::Vector3f* location,Eigen::Vector3f* euler){
  float x,y,z;
  std::stringstream ss(numStr);
  ss>>x>>y>>z;
  *location<<x,y,z;
  ss>>x>>y>>z;
  *euler<<x,y,z;
  std::cout<<"location: "<<location->x()<<" "<<location->y()<<" "<<location->z()<<" "<<std::endl;
  std::cout<<"euler: "<<euler->x()<<" "<<euler->y()<<" "<<euler->z()<<" "<<std::endl;
}

inline void World2oct::StringSize2Vector(std::string numStr,Eigen::Vector3f* size){
  float x,y,z;
  std::stringstream ss(numStr);
  ss>>x>>y>>z;
  *size<<x,y,z;
  std::cout<<"size: "<<size->x()<<" "<<size->y()<<" "<<size->z()<<" "<<std::endl;
}

//input the name of .world
bool World2oct::get_oct_from_xml(std::string Path){


  std::vector<Eigen::Vector3f> state_locations;
  std::vector<Eigen::Matrix3f> euler_matrixs;
  std::vector<Eigen::Vector3f> link_sizes;

  tinyxml2::XMLDocument* doc = new tinyxml2::XMLDocument();
  // std::string filePath = ros::package::getPath("model_gazebo");
  // filePath += "/world/";
  // filePath += Path;
  // doc->LoadFile(filePath.c_str());
  doc->LoadFile(Path.c_str());
  if(doc->ErrorID()){
    ROS_ERROR("cannot get the .world file from %s",Path.c_str());
    return false;
  }
  ROS_INFO("get the .world file from %s",Path.c_str());

  // tinyxml2::XMLElement* rootElement = doc->RootElement();
  // std::cout<<"Root Element:"<<rootElement->Value()<<std::endl;
  tinyxml2::XMLElement* worldElement = doc->RootElement()->FirstChildElement("world");
  tinyxml2::XMLElement* stateElement = worldElement->FirstChildElement("state");

  for(const tinyxml2::XMLElement* modelElement = worldElement->FirstChildElement("model"); modelElement; modelElement=modelElement->NextSiblingElement("model")){
    const tinyxml2::XMLAttribute* name = modelElement->FirstAttribute();
    std::string modelname = name->Value();
    printf( "get model name: %s\n", modelname.c_str());
    if(!strcmp(modelname.c_str(),"ground_plane"))
      continue;
    printf( "final model name: %s\n", modelname.c_str());

    //获取第一个model后的第一个pose
    Eigen::Vector3f model_location,model_euler;
    if(modelElement->FirstChildElement("pose")!=NULL){
      std::string pose_string = modelElement->FirstChildElement("pose")->GetText();
      ROS_INFO("model name: %s, pose: %s\n",modelname.c_str(),pose_string.c_str());  
      StringPose2Vector(pose_string,&model_location,&model_euler);
    }
    else{
      model_location << 0,0,0;
      model_euler << 0,0,0;
    }

    //读取link
    for(const tinyxml2::XMLElement* linkElement = modelElement->FirstChildElement("link"); linkElement; linkElement=linkElement->NextSiblingElement("link")){
      // Eigen::Vector3f link_location,link_euler;
      Eigen::Vector3f link_size;
      std::string link_name = linkElement->FirstAttribute()->Value();

      ROS_INFO("find link: %s",linkElement->FirstAttribute()->Value());
      const tinyxml2::XMLNode* first = linkElement->FirstChild();

      StringSize2Vector(linkElement->FirstChildElement("collision")->FirstChildElement("geometry")->FirstChildElement("box")->FirstChildElement("size")->GetText(),&link_size);

      bool state = false;
      Eigen::Vector3f state_location(0,0,0);
      Eigen::Vector3f state_euler(0,0,0);

      for(const tinyxml2::XMLElement* statemodelElement = stateElement->FirstChildElement("model"); statemodelElement; statemodelElement=statemodelElement->NextSiblingElement("model")){
        if(!strcmp(statemodelElement->FirstAttribute()->Value(),"ground_plane"))
          continue;
        for(const tinyxml2::XMLElement* statelinkElement = statemodelElement->FirstChildElement("link"); statelinkElement; statelinkElement=statelinkElement->NextSiblingElement("link")){
          std::string statelink_name = statelinkElement->FirstAttribute()->Value();
          if(!strcmp(link_name.c_str(),statelink_name.c_str())){
            state = true;
            ROS_INFO("from state:");
            StringPose2Vector(statelinkElement->FirstChildElement("pose")->GetText(),&state_location,&state_euler);
          }
          if(state)
            break;
        }
        if(state)
          break;
      }
      Eigen::Matrix3f euler_matrix;
      euler_matrix = Eigen::AngleAxisf(state_euler[2],Eigen::Vector3f::UnitZ())*
                     Eigen::AngleAxisf(state_euler[1],Eigen::Vector3f::UnitX())*
                     Eigen::AngleAxisf(state_euler[0],Eigen::Vector3f::UnitY());
      // insertbox(model_location+link_location+state_location, euler_matrix ,link_size);
      state_locations.push_back(state_location);
      euler_matrixs.push_back(euler_matrix);
      link_sizes.push_back(link_size);
      insertbox(state_location,euler_matrix,link_size);
    }


  }
  x_lower = floor(octomap_->getBBXMin().x() * inv_grid_interval_) * grid_interval_;
  y_lower = floor(octomap_->getBBXMin().y() * inv_grid_interval_) * grid_interval_;
  z_lower = floor(octomap_->getBBXMin().z() * inv_grid_interval_) * grid_interval_;
  x_upper = ceil(octomap_->getBBXMax().x() * inv_grid_interval_) * grid_interval_;
  y_upper = ceil(octomap_->getBBXMax().y() * inv_grid_interval_) * grid_interval_;
  z_upper = ceil(octomap_->getBBXMax().z() * inv_grid_interval_) * grid_interval_;

  

  GLX_SIZE = static_cast<int>(round((x_upper - x_lower) * inv_grid_interval_));
  GLY_SIZE = static_cast<int>(round((y_upper - y_lower) * inv_grid_interval_));
  GLZ_SIZE = static_cast<int>(round((z_upper - z_lower) * inv_grid_interval_)); 
  GLXYZ_SIZE = GLX_SIZE*GLY_SIZE*GLZ_SIZE;
  GLYZ_SIZE = GLY_SIZE*GLZ_SIZE;
  
  gridmap_ = new uint8_t[GLXYZ_SIZE];
  memset(gridmap_, 0, GLXYZ_SIZE * sizeof(uint8_t));


  for(int i=0;i<link_sizes.size();i++){
    grid_insertbox(state_locations[i],euler_matrixs[i],link_sizes[i]);
  }
  get_octomap_map_ = true;;
  get_grid_map_ = true;

  // std::cout<<"octomap_->getBBXMin()  Max()"<<octomap_->getBBXMin().z()<<" "<<octomap_->getBBXMax().z()<<std::endl;
  // std::cout<<"inv_grid_interval_"<<inv_grid_interval_<<std::endl;
  // std::cout<<"inv_grid_interval_"<<grid_interval_<<std::endl;
}

inline void World2oct::insertbox(Eigen::Vector3f location,Eigen::Matrix3f euler,Eigen::Vector3f size){
  Eigen::Vector3f x(1,0,0);
  Eigen::Vector3f y(0,1,0);
  Eigen::Vector3f z(0,0,1);
  x  = euler*x;
  y  = euler*y;
  z  = euler*z;
  float xmax,ymax,zmax;
  float xmin,ymin,zmin;
  xmax = octomap_->getBBXMax().x();
  ymax = octomap_->getBBXMax().y();
  zmax = octomap_->getBBXMax().z();
  xmin = octomap_->getBBXMin().x();
  ymin = octomap_->getBBXMin().y();
  zmin = octomap_->getBBXMin().z();
  float insert_interval = 0.5; 

  

  for(float i=-size.x()/2;i<=size.x()/2;i+=interval_*insert_interval)
    for(float j=-size.y()/2;j<=size.y()/2;j+=interval_*insert_interval)
      for(float k=-size.z()/2;k<=size.z()/2;k+=interval_*insert_interval){
        Eigen::Vector3f point = location+i*x+j*y+k*z;
        octomap_->updateNode(octomap::point3d(point.x(),point.y(),point.z()),true);
      }

  for(float i=-size.x()/2;i<=size.x()/2;i+=size.x())
    for(float j=-size.y()/2;j<=size.y()/2;j+=size.y())
      for(float k=-size.z()/2;k<=size.z()/2;k+=size.z()){
        Eigen::Vector3f point = location+i*x+j*y+k*z;
        if(point.x()>xmax)
          xmax = point.x();
        if(point.y()>ymax)
          ymax = point.y();
        if(point.z()>zmax)
          zmax = point.z();
        if(point.x()<xmin)
          xmin = point.x();
        if(point.y()<ymin)
          ymin = point.y();
        if(point.z()<zmin)
          zmin = point.z();
      }
  octomap::point3d BBXMAX(xmax,ymax,zmax);
  octomap::point3d BBXMIN(xmin,ymin,zmin);
  octomap_->setBBXMax(BBXMAX);
  octomap_->setBBXMin(BBXMIN);
}

inline void World2oct::grid_insertbox(Eigen::Vector3f location,Eigen::Matrix3f euler,Eigen::Vector3f size){
  Eigen::Vector3f x(1,0,0);
  Eigen::Vector3f y(0,1,0);
  Eigen::Vector3f z(0,0,1);
  x  = euler*x;
  y  = euler*y;
  z  = euler*z;

  float insert_interval = 0.5;
  for(float i=-size.x()/2;i<=size.x()/2;i+=grid_interval_*insert_interval)
    for(float j=-size.y()/2;j<=size.y()/2;j+=grid_interval_*insert_interval)
      for(float k=-size.z()/2;k<=size.z()/2;k+=grid_interval_*insert_interval){
        Eigen::Vector3f point = location+i*x+j*y+k*z;
        setObs(point);
      }
}

Eigen::Vector3f World2oct::gridIndex2coord(const Eigen::Vector3i &index){
  Eigen::Vector3f pt;
  pt(0) = ((float)index(0) + 0.5) * grid_interval_ + x_lower;
  pt(1) = ((float)index(1) + 0.5) * grid_interval_ + y_lower;
  pt(2) = ((float)index(2) + 0.5) * grid_interval_ + z_lower;
  return pt;
}

Eigen::Vector3i World2oct::coord2gridIndex(const Eigen::Vector3f &pt){
  Eigen::Vector3i idx;
  idx << std::min(std::max(int((pt(0) - x_lower) * inv_grid_interval_), 0), GLX_SIZE - 1),
      std::min(std::max(int((pt(1) - y_lower) * inv_grid_interval_), 0), GLY_SIZE - 1),
      std::min(std::max(int((pt(2) - z_lower) * inv_grid_interval_), 0), GLZ_SIZE - 1);
  return idx;
}

Eigen::Vector3i World2oct::coord2gridIndex(const Eigen::Vector3d &pt){
  Eigen::Vector3i idx;
  idx << std::min(std::max(int((pt(0) - x_lower) * inv_grid_interval_), 0), GLX_SIZE - 1),
      std::min(std::max(int((pt(1) - y_lower) * inv_grid_interval_), 0), GLY_SIZE - 1),
      std::min(std::max(int((pt(2) - z_lower) * inv_grid_interval_), 0), GLZ_SIZE - 1);
  return idx;
}

Eigen::Vector2i World2oct::coord2grid2dIndex(const Eigen::Vector2d &xypos){
  Eigen::Vector2i idx;
  idx << std::min(std::max(int((xypos(0) - x_lower) * inv_grid_interval_), 0), GLX_SIZE - 1),
      std::min(std::max(int((xypos(1) - y_lower) * inv_grid_interval_), 0), GLY_SIZE - 1);
  return idx;
}

void World2oct::setObs(const Eigen::Vector3f coord){
  float coord_x = coord.x();
  float coord_y = coord.y();
  float coord_z = coord.z();
  if (coord_x < x_lower || coord_y < y_lower || coord_z < z_lower ||
    coord_x >= x_upper || coord_y >= y_upper || coord_z >= z_upper)
    return;
  int idx_x = static_cast<int>((coord_x - x_lower) * inv_grid_interval_);
  int idx_y = static_cast<int>((coord_y - y_lower) * inv_grid_interval_);
  int idx_z = static_cast<int>((coord_z - z_lower) * inv_grid_interval_);
  gridmap_[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

Eigen::Vector3i World2oct::vectornum2gridIndex(const int num){
  Eigen::Vector3i index;
  index(0) = num / GLYZ_SIZE;
  index(1) = num % GLYZ_SIZE / GLZ_SIZE;
  index(2) = num % GLYZ_SIZE % GLZ_SIZE;
  return index;
}


void World2oct::TimeCallback(const ros::TimerEvent&){
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = "base";
  map_msg.header.stamp = ros::Time::now();
    
    
  ROS_INFO("waiting");
  if (octomap_msgs::fullMapToMsg(*(octomap_),map_msg)){
    ROS_INFO("success  1!");
    pub_octomap_.publish(map_msg);
    ROS_INFO("success  2!");
  }
  else
    ROS_ERROR("sredtyfugijoklmnjjbiojkmln bnvcgfy");
}

void World2oct::publish_octmap(){
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = "base";
  map_msg.header.stamp = ros::Time::now();
    
  if (octomap_msgs::fullMapToMsg(*(octomap_),map_msg)){
    pub_octomap_.publish(map_msg);
    ROS_INFO("octomap published!");
  }
  else
    ROS_ERROR("error octomap publish!");

  octomap::point3d max = octomap_->getBBXMax();
  octomap::point3d min = octomap_->getBBXMin();
  std::cout<<"BBXmax: "<<max.x()<<" "<<max.y()<<" "<<max.z()<<" "<<std::endl;
  std::cout<<"BBXmin: "<<min.x()<<" "<<min.y()<<" "<<min.z()<<" "<<std::endl; 
  std::cout<<"GLXYZ_SIZE:"<<GLXYZ_SIZE<<std::endl;
}

void World2oct::publish_gridmap(){
  ROS_INFO("publish grid_map!!");
  
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 map_vis;
  for(int idx = 1;idx < GLXYZ_SIZE;idx++){
    if(gridmap_[idx]==1){
      Eigen::Vector3f corrd = gridIndex2coord(vectornum2gridIndex(idx));
      pcl::PointXYZ pt(corrd.x(),corrd.y(),corrd.z());
      cloud_vis.points.push_back(pt);
    }
  }
  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, map_vis);
  map_vis.header.frame_id = "/base";
  pub_gridmap_.publish(map_vis);
}

uint8_t World2oct::CheckCollisionBycoord(const Eigen::Vector3f &pt){
  if(pt.x()>x_upper||pt.x()<x_lower||pt.y()>y_upper||pt.y()<y_lower||pt.z()>z_upper||pt.z()<z_lower){
    ROS_ERROR("[CheckCollisionBycoord], coord out of map!!!");
    return 255;
  }
  Eigen::Vector3i index = coord2gridIndex(pt);
  return gridmap_[index.x() * GLYZ_SIZE + index.y() * GLZ_SIZE + index.z()];
  
}
uint8_t World2oct::CheckCollisionBycoord(const Eigen::Vector3d &pt){
  if(pt.x()>x_upper||pt.x()<x_lower||pt.y()>y_upper||pt.y()<y_lower||pt.z()>z_upper||pt.z()<z_lower){
    ROS_ERROR("[CheckCollisionBycoord], coord out of map!!!");
    return 255;
  }
  Eigen::Vector3i index = coord2gridIndex(pt);
  return gridmap_[index.x() * GLYZ_SIZE + index.y() * GLZ_SIZE + index.z()];
}

uint8_t World2oct::CheckCollisionBycoord(const double ptx,const double pty,const double ptz){
  if(ptx>x_upper||ptx<x_lower||pty>y_upper||pty<y_lower||ptz>z_upper||ptz<z_lower){
    // ROS_INFO("\033[43;38m ptx:%f pty:%f ptz:%f  \033[0m",ptx,pty,ptz);
    ROS_ERROR("[CheckCollisionBycoord], coord out of map!!!");
    return 255;
  }
  Eigen::Vector3i index = coord2gridIndex(Eigen::Vector3d(ptx,pty,ptz));
  return gridmap_[index.x() * GLYZ_SIZE + index.y() * GLZ_SIZE + index.z()];
}

// 注意 surf没有考虑边界点  因为边界点检查四周碰撞会有bug
void World2oct::get_surf(){
  surf_id_.clear();
  surf_.clear();
  
  Eigen::Vector3i bound(GLXYZ_SIZE-GLYZ_SIZE,GLYZ_SIZE-GLZ_SIZE,GLZ_SIZE-1);
  Eigen::Vector3i boundinterval(GLYZ_SIZE,GLZ_SIZE,1);
  for(int x=0; x<=bound(0); x+=boundinterval(0)){
    for(int y=0; y<=bound(1); y+=boundinterval(1)){
      for(int z=0; z<=bound(2); z+=boundinterval(2)){
        // std::cout<<x<<"  "<<y<<"  "<<z<<std::endl;
        // if(x==0||x==bound(0)||y==0||y==bound(1)||z==0||z==bound(2)){
        //   // Eigen::Vector3i index(x/boundinterval(0),y/boundinterval(1),z/boundinterval(2));
        //   // surf_id_.push_back(index);
        //   // Eigen::Vector3f surf_point = gridIndex2coord(index);
        //   // surf_.emplace_back(surf_point.x(),surf_point.y(),surf_point.z());
        //   continue;
        // }
        // std::cout<<"x: "<<x<<"y: "<<y<<"z: "<<z<<std::endl;
        if(gridmap_[x + y + z] == Occupied){
          if( (x!=bound(0) && gridmap_[x+boundinterval(0) + y + z] == Unoccupied) || 
              (x!=0 && gridmap_[x-boundinterval(0) + y + z] == Unoccupied) || 
              (y!=bound(1) && gridmap_[x + y+boundinterval(1) + z] == Unoccupied) || 
              (y!=0 && gridmap_[x + y+boundinterval(1) + z] == Unoccupied) ||
              (z!=bound(2) && gridmap_[x + y + z+boundinterval(2)] == Unoccupied) ||
              (z!=0 && gridmap_[x + y + z+boundinterval(2)] == Unoccupied)){
              //  ROS_INFO("00");
               Eigen::Vector3i index(x/boundinterval(0),y/boundinterval(1),z/boundinterval(2));
              //  ROS_INFO("01");
               surf_id_.push_back(index);
              //  ROS_INFO("02");
               Eigen::Vector3f surf_point = gridIndex2coord(index);
              //  ROS_INFO("03");
               surf_.emplace_back(surf_point.x(),surf_point.y(),surf_point.z());
          }
          // ROS_INFO("1");
          if(x!=bound(0) && gridmap_[x+boundinterval(0) + y + z] == Unoccupied){
            Eigen::Vector3i index(x/boundinterval(0),y/boundinterval(1),z/boundinterval(2));
            surf_id_.push_back(index);
            Eigen::Vector3f surf_point = gridIndex2coord(index);
            surf_.emplace_back(surf_point.x()+0.5*grid_interval_,surf_point.y(),surf_point.z());
          }
          // ROS_INFO("2");
          if(x!=0 && gridmap_[x-boundinterval(0) + y + z] == Unoccupied){
            Eigen::Vector3i index(x/boundinterval(0),y/boundinterval(1),z/boundinterval(2));
            surf_id_.push_back(index);
            Eigen::Vector3f surf_point = gridIndex2coord(index);
            surf_.emplace_back(surf_point.x()-0.5*grid_interval_,surf_point.y(),surf_point.z());
          }
          // ROS_INFO("3");
          if(y!=bound(1) && gridmap_[x + y+boundinterval(1) + z] == Unoccupied){
            Eigen::Vector3i index(x/boundinterval(0),y/boundinterval(1),z/boundinterval(2));
            surf_id_.push_back(index);
            Eigen::Vector3f surf_point = gridIndex2coord(index);
            surf_.emplace_back(surf_point.x(),surf_point.y()+0.5*grid_interval_,surf_point.z());
          }
          // ROS_INFO("4");
          if(y!=0 && gridmap_[x + y-boundinterval(1) + z] == Unoccupied){
            Eigen::Vector3i index(x/boundinterval(0),y/boundinterval(1),z/boundinterval(2));
            surf_id_.push_back(index);
            Eigen::Vector3f surf_point = gridIndex2coord(index);
            surf_.emplace_back(surf_point.x(),surf_point.y()-0.5*grid_interval_,surf_point.z());
          }
          // ROS_INFO("5");
          if(z!=bound(2) && gridmap_[x + y + z+boundinterval(2)] == Unoccupied){
            Eigen::Vector3i index(x/boundinterval(0),y/boundinterval(1),z/boundinterval(2));
            surf_id_.push_back(index);
            Eigen::Vector3f surf_point = gridIndex2coord(index);
            surf_.emplace_back(surf_point.x(),surf_point.y(),surf_point.z()+0.5*grid_interval_);
          }
          // ROS_INFO("6");
          if(z!=0 && gridmap_[x + y + z-boundinterval(2)] == Unoccupied){
            Eigen::Vector3i index(x/boundinterval(0),y/boundinterval(1),z/boundinterval(2));
            surf_id_.push_back(index);
            Eigen::Vector3f surf_point = gridIndex2coord(index);
            surf_.emplace_back(surf_point.x(),surf_point.y(),surf_point.z()-0.5*grid_interval_);
          }

        }
      }
    }
  }
  ROS_INFO("surf_.size(): %d",surf_.size());
}

void World2oct::publish_surf(){
  ROS_INFO("publish surface!!");
  pcl::PointCloud<pcl::PointXYZ> cloud_vis;
  sensor_msgs::PointCloud2 surf_vis;
  int size = surf_.size();
  for(int i = 1; i< size;i++){
    pcl::PointXYZ pt(surf_[i].x(),surf_[i].y(),surf_[i].z());
    cloud_vis.points.push_back(pt);
  }
  cloud_vis.width = cloud_vis.points.size();
  cloud_vis.height = 1;
  cloud_vis.is_dense = true;
  pcl::toROSMsg(cloud_vis, surf_vis);
  surf_vis.header.frame_id = "/base";
  pub_surf_.publish(surf_vis);
}

bool World2oct::get_oct_from_yaml(){
  std::vector<float> boxes;
  nh_.getParam(ros::this_node::getName()+"/box", boxes);
  std::vector<double> boundary;
  nh_.getParam(ros::this_node::getName()+"/map_size", boundary);


  if(boxes.size()%7 != 0 || boundary.size()%6!=0){
    ROS_ERROR("boxes.size()%7!=0 || boundary,size()%6!=0");
  }

  octomap::point3d BBXMAX(boundary[1],boundary[3],boundary[5]);
  octomap_->setBBXMax(BBXMAX);
  octomap::point3d BBXMIN(boundary[0],boundary[2],boundary[4]);
  octomap_->setBBXMin(BBXMIN);

  std::cout<<BBXMAX.x()<<"   "<<BBXMAX.y()<<"   "<<BBXMAX.z()<<"   "<<std::endl;
  std::cout<<BBXMIN.x()<<"   "<<BBXMIN.y()<<"   "<<BBXMIN.z()<<"   "<<std::endl;

  std::vector<Eigen::Vector3f> state_locations;
  std::vector<Eigen::Matrix3f> euler_matrixs;
  std::vector<Eigen::Vector3f> link_sizes;

  for(int i=0; i<boxes.size()/7; i++){
    Eigen::Vector3f location(boxes[7*i],boxes[7*i+1],boxes[7*i+2]);
    Eigen::Vector3f size(boxes[7*i+3], boxes[7*i+4], boxes[7*i+5]);
    Eigen::Matrix3f euler(Eigen::AngleAxisf(boxes[7*i+6],Eigen::Vector3f::UnitZ()));
    state_locations.push_back(location);
    link_sizes.emplace_back(size);
    euler_matrixs.emplace_back(euler);

    // 更新octomap_
    Eigen::Vector3f x(1,0,0);
    Eigen::Vector3f y(0,1,0);
    Eigen::Vector3f z(0,0,1);
    x  = euler*x;
    y  = euler*y;
    z  = euler*z;
    float insert_interval = 0.5; 
    for(float i=-size.x()/2;i<=size.x()/2;i+=interval_*insert_interval)
      for(float j=-size.y()/2;j<=size.y()/2;j+=interval_*insert_interval)
        for(float k=-size.z()/2;k<=size.z()/2;k+=interval_*insert_interval){
          Eigen::Vector3f point = location+i*x+j*y+k*z;
          octomap_->updateNode(octomap::point3d(point.x(),point.y(),point.z()),true);
        }
  }
  octomap_->updateNode(octomap_->getBBXMax(),true);
  octomap_->updateNode(octomap_->getBBXMin(),true);

  x_lower = floor(octomap_->getBBXMin().x() * inv_grid_interval_) * grid_interval_;
  y_lower = floor(octomap_->getBBXMin().y() * inv_grid_interval_) * grid_interval_;
  z_lower = floor(octomap_->getBBXMin().z() * inv_grid_interval_) * grid_interval_;
  x_upper = ceil(octomap_->getBBXMax().x() * inv_grid_interval_) * grid_interval_;
  y_upper = ceil(octomap_->getBBXMax().y() * inv_grid_interval_) * grid_interval_;
  z_upper = ceil(octomap_->getBBXMax().z() * inv_grid_interval_) * grid_interval_;
  GLX_SIZE = static_cast<int>(round((x_upper - x_lower) * inv_grid_interval_));
  GLY_SIZE = static_cast<int>(round((y_upper - y_lower) * inv_grid_interval_));
  GLZ_SIZE = static_cast<int>(round((z_upper - z_lower) * inv_grid_interval_)); 
  GLXYZ_SIZE = GLX_SIZE*GLY_SIZE*GLZ_SIZE;
  GLYZ_SIZE = GLY_SIZE*GLZ_SIZE;
    
    
    
  gridmap_ = new uint8_t[GLXYZ_SIZE];
  memset(gridmap_, 0, GLXYZ_SIZE * sizeof(uint8_t));


  for(int i=0;i<link_sizes.size();i++){
    grid_insertbox(state_locations[i],euler_matrixs[i],link_sizes[i]);
  }
  get_octomap_map_ = true;;
  get_grid_map_ = true;
  ROS_INFO("from yaml init over!!!!!!!");
  sleep(1);
  
}


int main(int argc, char **argv){
  ros::init(argc, argv, "world2oct");
  ros::NodeHandle nh("~");
  
  float interval;
  float gridmap_interval;
  std::string file_name;
  nh.param<float>("/octomap/octomap_interval",interval,0.05);
  nh.param<float>("/octomap/gridmap_interval",gridmap_interval,0.1);
  



  World2oct* world2oct =  new World2oct(nh);
  // world2oct->get_oct_from_xml(file_name.c_str());
  // world2oct->publish_octmap();
  // world2oct->publish_gridmap();


  ros::spin();

  return 0;
}

