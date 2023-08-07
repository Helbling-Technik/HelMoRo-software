#pragma once

#include "helmoro_msgs/EditObjectMap.h"
#include "helmoro_msgs/ObjectDetections.h"
#include "helmoro_msgs/RequestObjectFromMap.h"
#include "ros/node_handle.h"
#include "ros/service_server.h"

// ROS
#include <ros/ros.h>
#include <sys/types.h>

// eigen
#include <eigen3/Eigen/Eigen>

// STL
#include <unordered_set>

namespace helmoro
{
class ObjectFilter
{
public:
  using vec3_t = Eigen::Vector3f;

  // constructor
  explicit ObjectFilter(vec3_t pos, ros::Time ts, float time_constant);
  ~ObjectFilter(void);

  // filter update
  void update(vec3_t pos, ros::Time ts);

  // public get functions
  float getDistance(vec3_t pos) const
  {
    return (pos - pos_).norm();
    // return ((pos - pos_).norm());
  };
  vec3_t getPos(void) const
  {
    return pos_;
  };
  uint32_t getUID(void) const
  {
    return uid_;
  };
  uint32_t getNumberOfDetections(void) const
  {
    return number_of_detections_;
  }

  // compare operator
  bool operator==(const ObjectFilter& rhs)
  {
    return getUID() == rhs.getUID();
  }

private:
  vec3_t pos_;
  uint32_t number_of_detections_;
  ros::Time ts_;
  uint32_t uid_;

  static uint32_t last_uid_;
  static std::unordered_set<uint32_t> set_uid_;

  // parameters
  float update_time_constant_;
};

class ObjectMap
{
public:
  using vec3_t = Eigen::Matrix<float, 3, 1>;

  // constructor
  explicit ObjectMap(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  explicit ObjectMap(void) : ObjectMap(ros::NodeHandle(), ros::NodeHandle("~")){};
  ~ObjectMap(void){};

  // public run functions
  void run(void);

private:
  // general variables
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // parameters
  float max_matching_dist_;
  float filter_time_constant_;
  float update_frequency_;
  int object_detection_threshold_;

  // object map
  std::map<int, std::vector<ObjectFilter>> object_map_;

  // publisher & subscriber
  ros::Subscriber sub_object_detections_;
  ros::Publisher pub_object_map_markers_;

  ros::ServiceServer serv_object_map_request_object_server_;
  ros::ServiceServer serv_object_map_edit_server_;
  // callbacks
  void objectDetectionsCallback(const helmoro_msgs::ObjectDetectionsConstPtr& msg);
  bool requestObjectFromMapCallback(helmoro_msgs::RequestObjectFromMapRequest& req,
                                    helmoro_msgs::RequestObjectFromMapResponse& res);
  bool editObjectMapCallback(helmoro_msgs::EditObjectMapRequest& req, helmoro_msgs::EditObjectMapResponse& res);
};
}  // namespace helmoro