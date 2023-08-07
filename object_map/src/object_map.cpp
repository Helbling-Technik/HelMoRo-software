#include "object_map/object_map.hpp"

// helmoro
#include "helmoro_msgs/EditObjectMap.h"
#include "ros/console.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

// STL
#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <vector>

namespace helmoro
{
//
// Object Filter
//

uint32_t ObjectFilter::last_uid_ = 0;
std::unordered_set<uint32_t> ObjectFilter::set_uid_;

// constructor
ObjectFilter::ObjectFilter(vec3_t pos, ros::Time ts, float time_constant) : update_time_constant_(time_constant)
{
  pos_ = pos;
  ts_ = ts;
  number_of_detections_ = 1;  // Constructor of item therefore first detection

  while (set_uid_.find(last_uid_) != set_uid_.end())
    ++last_uid_;
  uid_ = last_uid_++;
  set_uid_.insert(uid_);
}

ObjectFilter::~ObjectFilter(void)
{
  set_uid_.erase(uid_);
}

// filter update
void ObjectFilter::update(vec3_t pos, ros::Time ts)
{
  float dt = (ts - ts_).toSec();
  float c = exp(-dt / update_time_constant_);

  pos_ = (1.f - c) * pos + c * pos_;
  ts_ = ts;
  number_of_detections_++;  // Adding a datapoint to this object
}

//
// Object Map
//

// constructor
ObjectMap::ObjectMap(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
{
  // general variables
  nh_ = nh;
  pnh_ = pnh;

  // parameters
  pnh_.param<float>("/object_map/max_matching_distance", max_matching_dist_, 0.5);
  pnh_.param<float>("/object_map/filter_time_constant", filter_time_constant_, 0.5);
  pnh_.param<float>("/object_map/update_frequency", update_frequency_, 1);
  pnh_.param("/object_map/object_detection_threshold", object_detection_threshold_, 20);

  // publisher & subscriber
  sub_object_detections_ =
      nh_.subscribe("/object_detector/object_detections", 1, &ObjectMap::objectDetectionsCallback, this);
  pub_object_map_markers_ = pnh_.advertise<visualization_msgs::MarkerArray>("object_map_markers", 1);
  // Object request_service
  serv_object_map_request_object_server_ =
      nh_.advertiseService("/object_map/request_object", &ObjectMap::requestObjectFromMapCallback, this);
  // Map editing service
  serv_object_map_edit_server_ = nh_.advertiseService("/object_map/edit_map", &ObjectMap::editObjectMapCallback, this);
  // start main thread
  std::thread main_thread(&ObjectMap::run, this);
  main_thread.detach();
}

// public run functions
void ObjectMap::run(void)
{
  ros::Rate loop_rate(update_frequency_);
  while (ros::ok()) {
    visualization_msgs::MarkerArray object_map_marker_array_msg;
    // Defining parameters which are equal for all objects
    visualization_msgs::Marker object_marker;
    object_marker.header.stamp = ros::Time::now();
    // Note that the timestamp attached to the marker message above is ros::Time(), which is time Zero (0). This is
    // treated differently by RViz than any other time. If you use ros::Time::now() or any other non-zero value, rviz
    // will only display the marker if that time is close enough to the current time, where "close enough" depends on
    // TF. With time 0 however, the marker will be displayed regardless of the current time.
    object_marker.header.frame_id = "map";
    object_marker.ns = "object_map";
    object_marker.action = visualization_msgs::Marker::ADD;  // TODO currently due to the usage uid cubes are NOT
                                                             // deleted if they do not exist in the object_map
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.lifetime = ros::Duration(1.5);  // Time after which the marker disappears.

    // The pose is unknown therefore assuming no rotation
    object_marker.pose.orientation.x = 0.0;
    object_marker.pose.orientation.y = 0.0;
    object_marker.pose.orientation.z = 0.0;
    object_marker.pose.orientation.w = 1.0;
    float object_size = 0.07;  // in m
    object_marker.scale.x = object_size;
    object_marker.scale.y = object_size;
    object_marker.scale.z = object_size;

    for (auto object_filters : object_map_) {
      // Color conversion
      cv::Mat3f hsv(cv::Vec3f(object_filters.first * 2, 1.0, 1.0));  // H (0-360) S (0-1) V (0-1)
      cv::Mat3f rgb;
      cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);

      object_marker.color.r = rgb.at<float>(0);
      object_marker.color.g = rgb.at<float>(1);
      object_marker.color.b = rgb.at<float>(2);

      for (auto object_filter : object_filters.second) {
        if (object_filter.getNumberOfDetections() >= object_detection_threshold_) {  // only publish if max # detection
                                                                                     // is higher then threshold
          object_marker.color.a =
              0.2 + std::min((object_filter.getNumberOfDetections() - object_detection_threshold_) / 100.0 * 0.8,
                             0.8);  // alpha starts at 20% and is 100% if 100 detections are found
          object_marker.id = object_filter.getUID();
          object_marker.pose.position.x = object_filter.getPos().x();
          object_marker.pose.position.y = object_filter.getPos().y();
          object_marker.pose.position.z = object_filter.getPos().z();
          object_map_marker_array_msg.markers.push_back(object_marker);
        }
      }
    }

    // if (pub_object_map_markers_.getNumSubscribers() > 0)
    pub_object_map_markers_.publish(object_map_marker_array_msg);

    loop_rate.sleep();
  }
}

// service callbacks

bool ObjectMap::requestObjectFromMapCallback(helmoro_msgs::RequestObjectFromMapRequest& req,
                                             helmoro_msgs::RequestObjectFromMapResponse& res)

{
  int uid = req.uid;
  auto requested_color = object_map_.find(req.color);
  if (requested_color != object_map_.end()) {  // Does color exist in map
    if (!requested_color->second.empty()) {    // Does color list still have objects
      // Datatype from automatic extension of auto
      std::vector<ObjectFilter>::const_iterator requested_object;
      if (req.uid == -1) {
        requested_object = std::max_element(requested_color->second.begin(), requested_color->second.end(),
                                            // lambda expression beginn
                                            [](ObjectFilter a, ObjectFilter b) {
                                              return (a.getNumberOfDetections() < b.getNumberOfDetections());
                                            }  // end of lambda
        );
      } else {  // asked for specifc ID
        requested_object = std::find_if(requested_color->second.begin(), requested_color->second.end(),
                                        // lambda expression beginn
                                        [uid](ObjectFilter a) { return (a.getUID() == uid); }  // end of lambda
        );
        if (requested_object == requested_color->second.end()) {
          ROS_WARN("[Object Map]: Could not find requested Object with UID %d in Map", req.uid);
          return false;
        }
      }

      res.color = requested_color->first;
      res.uid = requested_object->getUID();
      res.point.x = requested_object->getPos().x();
      res.point.y = requested_object->getPos().y();
      res.point.z = requested_object->getPos().z();
      res.header.frame_id = "map";
      res.header.stamp = ros::Time::now();
      ROS_INFO("[Object Map]: Service Response with object at (%f, %f, %f), which has been detected %d times",
               res.point.x, res.point.y, res.point.z, requested_object->getNumberOfDetections());
      return true;
    }
  }
  return false;
}

bool ObjectMap::editObjectMapCallback(helmoro_msgs::EditObjectMapRequest& req, helmoro_msgs::EditObjectMapResponse& res)
{
  if (req.edit_command == 0) {
    ROS_WARN("[Object Map]: EditObjectMap request requested to do nothing, why was it send?");
    // Should this case exist in the service? no we can throw error message and then log info as long as the client does
    // not need to do anything with that information. currently it does not matter to the navigator if the call is
    // failed
    return false;
  } else if (req.edit_command == 1) {
    if (object_map_.at(req.color).empty()) {  // checking if the color exists in the map
      ROS_WARN("[Object Map]: EditObjectMap request: Color %d is not in Object Map", req.color);
      return false;
    }

    for (std::vector<ObjectFilter>::iterator iter = object_map_.at(req.color).begin();
         iter != object_map_.at(req.color).end(); std::advance(iter, 1)) {
      if (iter->getUID() == req.uid) {  // Deletes first element with this UID in this color list
        object_map_.at(req.color).erase(iter);
        ROS_INFO("[Object Map]: EditObjectMap request: requested deletion of object %d: successful", req.uid);
        return true;
      }
    }
    ROS_WARN("[Object Map]: EditObjectMap request:  object %d not in color list", req.uid);
    return false;
  }
  ROS_WARN("[Object Map]: EditObjectMap request: Edit Command %d is unknown", req.edit_command);
  return false;
}

// callbacks
void ObjectMap::objectDetectionsCallback(const helmoro_msgs::ObjectDetectionsConstPtr& msg)
{
  if (msg->num_detections_total != msg->color.size() || msg->num_detections_total != msg->point.size()) {
    std::printf("[%s] Received invalid message (num_detections_total not equal to color or point size).\n", __func__);
    return;
  }

  if (msg->num_detections_total == 0)
    return;

  // sort color by objects
  std::vector<int> indices(msg->num_detections_total);
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&](int& a, int& b) -> bool { return msg->color[a] < msg->color[b]; });

  // process objects groupwise by color
  int idx_start = 0;
  while (idx_start < indices.size()) {
    int idx_end = idx_start + 1;
    while (idx_end < indices.size() && msg->color[indices[idx_start]] == msg->color[indices[idx_end]])
      ++idx_end;

    // iterate over all objects from the same color
    uint8_t color = msg->color[indices[idx_start]];
    if (object_map_[color].empty()) {
      // new color, add all object
      for (int i = idx_start; i < idx_end; i++) {
        vec3_t pos(msg->point[indices[i]].x, msg->point[indices[i]].y, msg->point[indices[i]].z);
        object_map_[color].emplace_back(pos, msg->header.stamp, filter_time_constant_);
      }
    } else {
      // objects of this color exist, check before adding new object
      int j_max = object_map_[color].size();

      // compute cost for each object to each filter
      std::map<int, std::vector<float>> cost;
      for (int i = idx_start; i < idx_end; ++i) {
        vec3_t pos(msg->point[indices[i]].x, msg->point[indices[i]].y, msg->point[indices[i]].z);

        for (int j = 0; j < j_max; ++j) {
          cost[i].push_back(object_map_[color][j].getDistance(pos));
        }
      }

      // match the ones with the lowest cost
      std::unordered_set<int> set_i, set_j;
      for (int i = idx_start; i < idx_end; ++i)
        set_i.insert(i);
      for (int j = 0; j < j_max; ++j)
        set_j.insert(j);

      while (!set_i.empty() && !set_j.empty()) {
        float best_cost = std::numeric_limits<float>::max();
        int best_i = -1, best_j = -1;

        for (auto i : set_i) {
          for (auto j : set_j) {
            if (cost[i][j] < best_cost) {
              best_cost = cost[i][j];
              best_i = i;
              best_j = j;
            }
          }
        }

        if (best_cost < max_matching_dist_) {
          vec3_t pos(msg->point[indices[best_i]].x, msg->point[indices[best_i]].y, msg->point[indices[best_i]].z);
          object_map_[color][best_j].update(pos, msg->header.stamp);

          // remove entries from sets
          set_i.erase(best_i);
          set_j.erase(best_j);
        } else {
          break;
        }
      }

      // add all other (unmatched) measurements
      for (auto i : set_i) {
        vec3_t pos(msg->point[indices[i]].x, msg->point[indices[i]].y, msg->point[indices[i]].z);
        object_map_[color].emplace_back(pos, msg->header.stamp, filter_time_constant_);
      }
    }

    // done
    idx_start = idx_end;
  }
}

}  // namespace helmoro