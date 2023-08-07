#pragma once

// STL
#include <mutex>

// any_node
#include "any_node/Node.hpp"
#include "sensor_msgs/image_encodings.h"

// ROS
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

//#include <sensor_msgs/PointCloud.h>

// eigen
#include <eigen3/Eigen/Eigen>

// OpenCV
#include <opencv2/opencv.hpp>

namespace helmoro
{
class ObjectDetector /*: public any_node::Node*/
{
public:
  using mat3x3_t = Eigen::Matrix<float, 3, 3>;
  using vec3_t = Eigen::Matrix<float, 3, 1>;

  // constructor
  explicit ObjectDetector(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  explicit ObjectDetector(void) : ObjectDetector(ros::NodeHandle(), ros::NodeHandle("~")){};
  ~ObjectDetector(void);

  void staticPointCloud();

private:
  // general variables
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // real robot check
  bool is_simulation_{false};
  std::string depth_encoding_{sensor_msgs::image_encodings::TYPE_16UC1};
  float depth_unit_conversion_factor_{0.001f};

  // color range
  std::vector<int> hue_;
  std::vector<std::vector<int>> hsv_low_;
  std::vector<std::vector<int>> hsv_high_;

  // depth computation
  int mean_block_half_width_;

  // counter for verifying
  int object_detection_valid_after_;
  int object_detector_counter_;

  // geometric object detection threshold
  float max_object_height_;

  // rgb camera
  bool cam_initialized_{false};
  
  
  void updateCameraParameters(const sensor_msgs::CameraInfoConstPtr& msg);

  // publisher & subscriber
  image_transport::Subscriber sub_image_;
  image_transport::Subscriber sub_depth_;
  image_transport::Publisher pub_object_detection_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener sub_tf_;
  ros::Publisher pub_object_positions_;

  ros::Subscriber sub_cloud_moment_;
  ros::Publisher pub_cloud_moment_;

  sensor_msgs::PointCloud2 cloud_in_;

  // callbacks
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void depthCallback(const sensor_msgs::ImageConstPtr& msg);

  // depth map
  std::mutex depth_m_;
  cv::Mat depth_;

  // camera
  mat3x3_t cam_intrinsics_;
  mat3x3_t cam_intrinsics_inv_;
  vec3_t camera_position_vector_;
  vec3_t compute3DPoint(cv::Point2f uv, float depth);
};
}  // namespace helmoro