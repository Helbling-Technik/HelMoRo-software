#pragma once

// message logger
#include "geometry_msgs/Pose.h"
#include "message_logger/message_logger.hpp"

// any_node
#include "any_node/Node.hpp"

// Helmoro
#include "helmoro_description/enums/enums.hpp"
#include "helmoro_msgs/ObjectDetections.h"
#include "helmoro_msgs/ObjectMap.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Joy.h"

// ros
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

namespace helmoro_navigation_goals
{
enum class StateEnum : unsigned int {
  kWaiting = 0,
  kSelectingObjectFromMap = 1,
  kCommandPreparationGoal = 2,
  kNavigatingToPreparationGoal = 3,
  kRequestingToDeleteObject = 4,
  kCommandObjectGoal = 5,
  kNavigatingToObject = 6,
  kCommandTurningToGoal = 7,
  kTurningToGoal = 8,
  kCommandGlobalGoal = 9,
  kNavigatingToGlobalGoal = 10,
};

struct object_target {
  int uid;
  unsigned int color;
  move_base_msgs::MoveBaseGoal object_location;
};

class HelmoroNavigationGoals : public any_node::Node
{
public:
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  HelmoroNavigationGoals(NodeHandlePtr nh);
  virtual ~HelmoroNavigationGoals();

  virtual bool init();
  virtual void cleanup();
  virtual bool update(const any_worker::WorkerEvent& event);
  void ReadParameters();
  void UpdateStateMachine();
  void GetHelmoroPosition();
  geometry_msgs::Pose CalculateSetbackPose(geometry_msgs::Point first_position, geometry_msgs::Point second_position,
                                           double setback_dist);
  double SetMoveBaseParams(std::string name, double value);

  // Get most often seen object of this color
  void GetObjectPosition(unsigned int color);
  // Get object of this color with this uid
  void GetObjectPosition(unsigned int color, int uid);

  void objPointCallback(const helmoro_msgs::ObjectMapConstPtr& msg);
  void joystickCallback(const sensor_msgs::JoyConstPtr& msg);

private:
  StateEnum state_;

  // ros
  ros::Subscriber sub_obj_point_;
  ros::Subscriber sub_obj_map_;
  ros::Subscriber sub_joystick_commands_;
  ros::ServiceClient client_obj_map_edit_;
  ros::ServiceClient client_request_object_from_map_;
  ros::ServiceClient client_clear_costmaps_;

  MoveBaseClient action_client_;
  move_base_msgs::MoveBaseGoal global_goal_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener sub_tf_;

  ros::Subscriber sub_cloud_moment_;
  ros::Publisher pub_cloud_moment_;

  sensor_msgs::PointCloud2 cloud_in_;
  object_target current_object_;

  // object detection and targeting
  bool start_collecting_objects_command_{false};
  bool cancel_collecting_objects_command_{false};
  unsigned int selected_color_{400};  // defaults to a Hue value, which is invalid
  int loop_counter_{0};

  // helmoro's position
  geometry_msgs::TransformStamped helmoro_tf_;

  // external params for object targeting
  double setback_dist_{0};
  double drive_into_object_distance_{0};
  int target_mode_{0};
  double distance_fork_from_base_{0};
  double min_speed_transporting_object_{0};
  double xy_goal_tolerance_during_turn_with_object_{0};
  double previous_min_vel_x_{0};
  double previous_xy_goal_tolerance_{0};
};

}  // namespace helmoro_navigation_goals
