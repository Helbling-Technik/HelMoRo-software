#pragma once

// message logger
#include "message_logger/message_logger.hpp"

// any_node
#include "any_node/Node.hpp"

// helmoro
#include "helmoro_description/enums/enums.hpp"

// ros
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// general
#include <math.h>
#include <string>

namespace helmoro_local_obstacle {

class HelmoroLocalObstacle : public any_node::Node
{
 public:
    HelmoroLocalObstacle(NodeHandlePtr nh);
    virtual ~HelmoroLocalObstacle();

    virtual bool init();
    virtual void cleanup();
    virtual bool update(const any_worker::WorkerEvent& event);
    
    //void pointCloudCallback(const sensor_msgs::PointCloud2 cloud_msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    //void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_msg);
    //void transform_to_chassis(const sensor_msgs::PointCloud2ConstPtr& cloud_input, const sensor_msgs::PointCloud2ConstPtr& cloud_output);
    void cutPointCloud();

 private:
    ros::Subscriber pointCloudSub_;
    ros::Publisher pointCloudPub_;

    sensor_msgs::PointCloud2 cloud_in_;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_;
    sensor_msgs::PointCloud2 cloud_filtered_;

    Eigen::Affine3f transform_1_;
    Eigen::Affine3f transform_2_;

    double theta_ = M_PI/12;
    pcl::PassThrough<pcl::PointXYZ> pass_y_; //pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pcl::PassThrough<pcl::PointXYZ> pass_z_; //pcl::PassThrough<pcl::PointXYZRGB> pass_z;
    //pcl::PassThrough<pcl::PCLPointCloud2> pass_y2_;
    //pcl::PassThrough<pcl::PCLPointCloud2> pass_z2_;
};

} //helmoro_local_obstacle 
