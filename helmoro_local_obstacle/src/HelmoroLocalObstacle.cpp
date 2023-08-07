#include "helmoro_local_obstacle/HelmoroLocalObstacle.hpp"

namespace helmoro_local_obstacle {

HelmoroLocalObstacle::HelmoroLocalObstacle(NodeHandlePtr nh):
    any_node::Node(nh)
    //tf_listener_()
{
}

HelmoroLocalObstacle::~HelmoroLocalObstacle(){

}

bool HelmoroLocalObstacle::init() {
  //initialize publishers and subscribers 
  pointCloudSub_ = getNodeHandle().subscribe("/camera/depth/points", 1, &HelmoroLocalObstacle::pointCloudCallback, this);
  pointCloudPub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2> ("/pointcloud_filtered", 1, this);


  cloud_in_.header.seq = 0;

  //define update frequency
  auto workerTimeStep = param<double>("time_step_pub", 0.2);
  constexpr int priority = 0;
  addWorker("HelmoroLocalObstacle::updateWorker", workerTimeStep, &HelmoroLocalObstacle::update, this, priority);
  return true;
}

void HelmoroLocalObstacle::cleanup() {

}

bool HelmoroLocalObstacle::update(const any_worker::WorkerEvent& event) {

  if(cloud_in_.header.seq != 0){
    cutPointCloud();
  }
  //pointCloudPub_.publish(cloud_in_);
  return true;
}


void HelmoroLocalObstacle::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {  

  cloud_in_ = *cloud_msg;

}

void HelmoroLocalObstacle::cutPointCloud(){
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(cloud_in_, *cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.1, 0.1, 0.1);
  sor.filter(cloud_filtered);

  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);
  //output.header.frame_id = "camera_depth_optical_frame";
  pointCloudPub_.publish(output);

}


} //namespace helmoro_local_obstacle
