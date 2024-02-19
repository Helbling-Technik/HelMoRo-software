/**
 * @file HelmoroNavigationGoals.cpp
 * @brief Implementation of the HelmoroNavigationGoals class.
 */

#include "helmoro_navigation_goals/HelmoroNavigationGoals.hpp"

// STL
#include <boost/iterator/iterator_concepts.hpp>
#include <cmath>
#include <string>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "helmoro_msgs/EditObjectMap.h"
#include "helmoro_msgs/RequestObjectFromMap.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "ros/assert.h"
#include "ros/init.h"
#include "std_srvs/Empty.h"
#include "tf/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"

// ROS
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

namespace helmoro_navigation_goals
{
/**
 * @brief Constructor for HelmoroNavigationGoals class.
 *
 * @param nh NodeHandle pointer.
 */
HelmoroNavigationGoals::HelmoroNavigationGoals(NodeHandlePtr nh)
  : any_node::Node(nh), action_client_("move_base"), tf_buffer_(), sub_tf_(tf_buffer_)
{
}

/**
 * @brief Destructor for HelmoroNavigationGoals class.
 */
HelmoroNavigationGoals::~HelmoroNavigationGoals()
{
}

/**
 * @brief Initializes the HelmoroNavigationGoals class.
 *
 * @return true if initialization is successful, false otherwise.
 */
bool HelmoroNavigationGoals::init()
{
  sub_joystick_commands_ =
      getNodeHandle().subscribe("/helmoro_joystick", 1, &HelmoroNavigationGoals::joystickCallback, this);

  client_obj_map_edit_ = getNodeHandle().serviceClient<helmoro_msgs::EditObjectMap>("/object_map/edit_map");

  client_request_object_from_map_ = getNodeHandle().serviceClient<helmoro_msgs::RequestObjectFromMap>("/object_map/"
                                                                                                      "request_object");

  client_clear_costmaps_ = getNodeHandle().serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  // initialize variables
  state_ = StateEnum::kWaiting;

  ReadParameters();
  ros::Duration(1.0).sleep();
  GetHelmoroPosition();

  // wait for action server
  while (!action_client_.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("[Navigation Goals]: Waiting for move_base action server to come up");
  }

  auto workerTimeStep = param<double>("time_step_pub", 0.1);
  constexpr int priority = 0;
  addWorker("HelmoroNavigationGoals::updateWorker", workerTimeStep, &HelmoroNavigationGoals::update, this, priority);

  ROS_INFO("[Navigation Goals]: Init done!");
  return true;
}

/**
 * @brief Cleans up the HelmoroNavigationGoals class.
 */
void HelmoroNavigationGoals::cleanup()
{
}

/**
 * @brief Updates the HelmoroNavigationGoals class.
 *
 * @param event WorkerEvent object.
 * @return true if update is successful, false otherwise.
 */
bool HelmoroNavigationGoals::update(const any_worker::WorkerEvent& event)
{
  StateEnum old_state = state_;
  GetHelmoroPosition();
  UpdateStateMachine();

  int print_frequency = 10;

  switch (state_) {
    ////////////////////////////////// CASE: WAITING ////////////////////////////////////////////////
    case StateEnum::kWaiting:
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  waiting");
      break;

    ////////////////////////////////// CASE: SELECTING OBJECT FROM MAP //////////////////////////////
    case StateEnum::kSelectingObjectFromMap: {  // Scope so that temporary variables can be defined
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  PickingObject");

      // service call to get object

      // Get object from map without specifingID: therefore most often seen.
      GetObjectPosition(selected_color_);

      break;
    }
    ////////////////////////////////// CASE: COMMANDING PREPARATION GOAL /////////////////////////////
    case StateEnum::kCommandPreparationGoal: {  // for temp variables
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  kCommandToPreparationGoal");

      move_base_msgs::MoveBaseGoal preparation_goal;

      if (target_mode_ == 1) {
        // placing behind object
        preparation_goal.target_pose.pose =
            CalculateSetbackPose(current_object_.object_location.target_pose.pose.position,
                                 global_goal_.target_pose.pose.position, setback_dist_);
      } else if (target_mode_ == 2) {
        // placing in front of object
        geometry_msgs::Point current_helmoro_position;
        current_helmoro_position.x = helmoro_tf_.transform.translation.x;
        current_helmoro_position.y = helmoro_tf_.transform.translation.y;

        preparation_goal.target_pose.pose = CalculateSetbackPose(
            current_object_.object_location.target_pose.pose.position, current_helmoro_position, -setback_dist_);
      }

      preparation_goal.target_pose.header.frame_id = "map";
      preparation_goal.target_pose.header.stamp = ros::Time(0);

      ROS_WARN("[Navigation Goals]: Going to pick up object %d", current_object_.uid);
      action_client_.sendGoal(preparation_goal);
      state_ = StateEnum::kNavigatingToPreparationGoal;
      break;
    }
    ////////////////////////////////// CASE: NAVIGATING TO PREPARATION GOAL /////////////////////////
    case StateEnum::kNavigatingToPreparationGoal:
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  NavigatingToPreparationGoal");
      break;

      ////////////////////////////////// CASE: REQUESTING TO DELETE OBJECT///////////////////
    case StateEnum::kRequestingToDeleteObject: {
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  Requesting To Delete Object %d of color %d", current_object_.uid,
                    current_object_.color);

      // Get latest Information about the position of the object
      GetObjectPosition(current_object_.color, current_object_.uid);
      // Requesting to delete object as we are getting it now
      helmoro_msgs::EditObjectMap srv;
      std_srvs::Empty srv_clear_costmap;
      srv.request.uid = current_object_.uid;
      srv.request.color = current_object_.color;
      srv.request.edit_command = helmoro_msgs::EditObjectMap::Request::DELETE;

      // when service call finished change state
      if (client_obj_map_edit_.call(srv)) {
        // Requesting to clear costmap, so  object pickup is easier
        if (client_clear_costmaps_.call(srv_clear_costmap)) {
          ROS_INFO("[Navigation Goals]: Cleared CostMap");
          state_ = StateEnum::kCommandObjectGoal;
        } else {
          if (loop_counter_ < 5)
            ROS_ERROR("[Navigation Goals]: Could not clear costmap, trying again %d/5", loop_counter_ + 1);
          else {
            ROS_ERROR("[Navigation Goals]: Could not clear costmap after 5 tries, canceling");
            state_ = StateEnum::kWaiting;
          }
        }
      } else {
        if (loop_counter_ < 5)
          ROS_ERROR("[Navigation Goals]: Could not edit map, trying again %d/5", loop_counter_ + 1);
        else {
          ROS_ERROR("[Navigation Goals]: Could not edit map after 5 tries, canceling");
          state_ = StateEnum::kWaiting;
        }
      }

      break;
    }

    ////////////////////////////////// CASE: COMMANDING OBJECT GOAL //////////////////////////////////
    case StateEnum::kCommandObjectGoal: {
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  Command Object Goal");

      move_base_msgs::MoveBaseGoal pickup_goal;

      if (target_mode_ == 1) {
        pickup_goal.target_pose.pose =
            CalculateSetbackPose(current_object_.object_location.target_pose.pose.position,
                                 global_goal_.target_pose.pose.position, distance_fork_from_base_);
      } else if (target_mode_ == 2) {
        // placing in front of object
        geometry_msgs::Point current_helmoro_position;
        current_helmoro_position.x = helmoro_tf_.transform.translation.x;
        current_helmoro_position.y = helmoro_tf_.transform.translation.y;

        pickup_goal.target_pose.pose = CalculateSetbackPose(
            current_object_.object_location.target_pose.pose.position, current_helmoro_position,
            -(distance_fork_from_base_ - drive_into_object_distance_));  // 7cm is choosen as error margin in distance
                                                                         // detection
      }
      pickup_goal.target_pose.header.frame_id = "map";
      pickup_goal.target_pose.header.stamp = ros::Time(0);

      action_client_.sendGoal(pickup_goal);
      state_ = StateEnum::kNavigatingToObject;
      break;
    }
    ////////////////////////////////// CASE: NAVIGATING TO OBJECT /////////////////////////
    case StateEnum::kNavigatingToObject:
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  Navigating To Object");
      break;

    ////////////////////////////////// CASE: COMMAND TURNING WITH OBJECT //////////////////
    case StateEnum::kCommandTurningToGoal: {
      ROS_INFO_ONCE("[Navigation Goals]: STATE:   Commanding to turn to global goal");

      /// Setting the minimum_speed
      previous_min_vel_x_ = HelmoroNavigationGoals::SetMoveBaseParams("min_vel_x", min_speed_transporting_object_);
      previous_xy_goal_tolerance_ =
          HelmoroNavigationGoals::SetMoveBaseParams("xy_goal_tolerance", xy_goal_tolerance_during_turn_with_object_);

      ROS_WARN("[Navigation Goals]: Sending turning goal");
      move_base_msgs::MoveBaseGoal turning_goal;
      turning_goal.target_pose.header.frame_id = "map";
      turning_goal.target_pose.header.stamp = ros::Time(0);

      geometry_msgs::Point current_helmoro_position;
      current_helmoro_position.x = helmoro_tf_.transform.translation.x;
      current_helmoro_position.y = helmoro_tf_.transform.translation.y;

      turning_goal.target_pose.pose =
          CalculateSetbackPose(current_helmoro_position, global_goal_.target_pose.pose.position, 0);

      action_client_.sendGoal(turning_goal);
      state_ = StateEnum::kTurningToGoal;
      break;
    }
    ////////////////////////////////// CASE: TURNING WITH OBJECT /////////////////////////
    case StateEnum::kTurningToGoal:
      ROS_INFO_ONCE("[Navigation Goals]: STATE:   Turning to global goal");
      break;

    ////////////////////////////////// CASE: COMMANDING GLOBAL GOAL /////////////////////////
    case StateEnum::kCommandGlobalGoal:
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  Command Global Goal");

      ROS_WARN("[Navigation Goals]: Sending global goal");
      action_client_.sendGoal(global_goal_);
      state_ = StateEnum::kNavigatingToGlobalGoal;

    ////////////////////////////////// CASE: NAVIGATING TO GLOBAL GOAL /////////////////////////
    case StateEnum::kNavigatingToGlobalGoal:
      ROS_INFO_ONCE("[Navigation Goals]: STATE:  Navigating To Global Goal");
      break;
  }

  loop_counter_++;
  if (state_ != old_state)
    loop_counter_ = 0;

  return true;

}  // update

/**
 * @brief Updates the state machine.
 */
void HelmoroNavigationGoals::UpdateStateMachine()
{
  // Check first if collection of objects should be stopped
  if (cancel_collecting_objects_command_) {
    // Go  to Waiting state
    state_ = StateEnum::kWaiting;
    // cancel action server
    action_client_.cancelAllGoals();
  };
  // Updating statemachine with external commands
  switch (state_) {
    case StateEnum::kWaiting:  // Check if user input is given. If so, switch to kNavigatingToObject

      if (start_collecting_objects_command_) {
        state_ = StateEnum::kSelectingObjectFromMap;
        break;
      }

    case StateEnum::kSelectingObjectFromMap:
      break;  // Transitions after 1 loop in the SM

    case StateEnum::kCommandPreparationGoal:
      break;  // Transitions after 1 loop in the SM

    case StateEnum::kNavigatingToPreparationGoal:
      // Check if object target is reached successfully. If so, switch to
      // kRequestingToDeleteObject
      if (action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("[Navigation Goals]: Preparation Goal reached!");
        state_ = StateEnum::kRequestingToDeleteObject;
      }
      break;

    case StateEnum::kRequestingToDeleteObject:
      break;  // Transitions after 1 loop in the SM.

    case StateEnum::kCommandObjectGoal:
      break;

    case StateEnum::kNavigatingToObject:  // Check if object is picked up. If so, switch to kNavigatingToGoal
      if (action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("[Navigation Goals]: Picked up object!");
        if (target_mode_ == 1) {  // in target_mode 1 Helmoro is already behind the object: no turning required
          state_ = StateEnum::kCommandGlobalGoal;
          break;
        } else if (target_mode_ == 2) {
          state_ = StateEnum::kCommandTurningToGoal;
        } else {
          ROS_ERROR("[Navigation Goals]: Something went wrong with the target mode");
        }
        break;
      }
    case StateEnum::kCommandTurningToGoal:
      break;  // Transitions after 1 loop in the SM

    case StateEnum::kTurningToGoal:
      if (action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("[Navigation Goals]: Turned around with object");

        state_ = StateEnum::kCommandGlobalGoal;
      }
      break;

    case StateEnum::kCommandGlobalGoal:
      break;  // Transitions after 1 loop in the SM

    case StateEnum::kNavigatingToGlobalGoal:  // Check if Global Goal Position is reached. If so, switch to kWaiting
      if (action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[Navigation Goals]: Global goal reached!");

        // Reseting min_vel_x to the value it had before after goal is reached
        SetMoveBaseParams("min_vel_x", previous_min_vel_x_);
        SetMoveBaseParams("xy_goal_tolerance", previous_xy_goal_tolerance_);
        state_ = StateEnum::kWaiting;
      }
      break;
  }
}

/**
 * @brief Reads parameters from parameter server.
 */
void HelmoroNavigationGoals::ReadParameters()
{
  // Finding distance to fork from base
  geometry_msgs::TransformStamped tf_stamped;
  bool fork_frame_received = false;

  ros::Rate r(1);
  while (getNodeHandle().ok() && not fork_frame_received) {
    try {
      tf_stamped = tf_buffer_.lookupTransform("base_link", "fork_frame", ros::Time(0));
      fork_frame_received = true;
    } catch (tf2::TransformException& ex) {
      ROS_INFO("[Navigation Goals]: Waiting for transforms...");
      fork_frame_received = false;
    }
    ros::spinOnce();
    r.sleep();
  }

  distance_fork_from_base_ = tf_stamped.transform.translation.x;
  ROS_WARN("[Navigation Goals]: Received transforms: distance from base to fork is: %f", distance_fork_from_base_);

  // Setback distance behind the object
  setback_dist_ = param<double>("object_pickup/setback_distance", 0.35);
  drive_into_object_distance_ = param<double>("object_pickup/drive_into_object_distance_", 0.07);
  // Configure type of picking  up object
  target_mode_ = param<int>("object_pickup/target_mode", 1);
  ROS_INFO("[Helmoro Navigation rules]: Target Mode %d selected", target_mode_);
  // Minimum speed when transporting object can be negative
  min_speed_transporting_object_ = param<double>("object_pickup/min_speed_during_object_transport", 0.0);
  xy_goal_tolerance_during_turn_with_object_ =
      param<double>("object_pickup/xy_goal_tolerance_during_turn_with_object", 0.0);
  nh_->getParam("/move_base/DWAPlannerROS/min_vel_x", previous_min_vel_x_);
  nh_->getParam("/move_base/DWAPlannerROS/xy_goal_tolerance", previous_xy_goal_tolerance_);

  // Reading in position of final goal for all blocks
  global_goal_.target_pose.header.frame_id = param<std::string>("global_goal/header/frame_id", "map");
  global_goal_.target_pose.pose.position.x = param<double>("global_goal/pose/position/x", 0.0);
  global_goal_.target_pose.pose.position.y = param<double>("global_goal/pose/position/y", 0.0);
  global_goal_.target_pose.pose.position.z = param<double>("global_goal/pose/position/z", 0.0);
  global_goal_.target_pose.pose.orientation.x = param<double>("global_goal/pose/orientation/x", 0.0);
  global_goal_.target_pose.pose.orientation.y = param<double>("global_goal/pose/orientation/y", 0.0);
  global_goal_.target_pose.pose.orientation.z = param<double>("global_goal/pose/orientation/z", 0.0);
  global_goal_.target_pose.pose.orientation.w = param<double>("global_goal/pose/orientation/w", 1.0);

  ROS_INFO("[Navigation Goals]: Global goal at x: %f, y: %f, z: %f", global_goal_.target_pose.pose.position.x,
           global_goal_.target_pose.pose.position.y, global_goal_.target_pose.pose.position.z);
}

/**
 * @brief Function which calculates a pose which is along the line of 2 positions, but setback
 *
 * @param first_position The first position along the line, behind which the pose is set
 * @param second_position The second position to which the line goes
 * @param setback_dist Distance behind first_position along the line between first_position and two_position
 * @return geometry_msgs::Pose  The resulting pose including yaw orientation
 */
geometry_msgs::Pose HelmoroNavigationGoals::CalculateSetbackPose(geometry_msgs::Point first_position,
                                                                 geometry_msgs::Point second_position,
                                                                 double setback_dist)
{
  double setback_x;
  double setback_y;

  double delta_x = second_position.x - first_position.x;
  double delta_y = second_position.y - first_position.y;
  double yaw_orientation_to_second = atan2(delta_y, delta_x);

  setback_x = -setback_dist * cos(yaw_orientation_to_second);
  setback_y = -setback_dist * sin(yaw_orientation_to_second);

  geometry_msgs::Pose setback_pose;

  setback_pose.position.x = first_position.x + setback_x;
  setback_pose.position.y = first_position.y + setback_y;
  setback_pose.position.z = 0;
  // Orientation is always to the object, therefore the negative of the setbacks arctan
  setback_pose.orientation = tf::createQuaternionMsgFromYaw(atan2(-setback_y, -setback_x));

  return setback_pose;
}

/**
 * @brief sets a parameter of the move_base node
 *
 * @param name name of the parameter
 * @param value value of the parameter
 *
 * @return old value of the parameter
 */
double HelmoroNavigationGoals::SetMoveBaseParams(std::string name, double value)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  double old_value;

  nh_->getParam("/move_base/DWAPlannerROS/" + name, old_value);

  // Setting min_vel_x of move_base
  double_param.name = name;
  double_param.value = value;
  conf.doubles.push_back(double_param);

  srv_req.config = conf;
  // sending the parametere to the dynamic reconfigure server
  ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

  // ROS_INFO_STREAM("[Helmoro Navigation Goals]: Setting name: \n " << srv_resp.config);
  return old_value;
}

/**
 * @brief gets the current position of the helmoro
 *
 * @return void
 */
void HelmoroNavigationGoals::GetHelmoroPosition()
{
  try {
    helmoro_tf_ = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
}

/**
 * @brief gets the position of an object from the object map
 *
 * @param color color of the object
 * @param uid uid of the object
 *
 * @return void
 */
void HelmoroNavigationGoals::GetObjectPosition(unsigned int color, int uid)
{
  // service call to get object
  helmoro_msgs::RequestObjectFromMap srv;
  srv.request.color = color;
  srv.request.uid = uid;

  if (client_request_object_from_map_.call(srv)) {
    current_object_.object_location.target_pose.header.frame_id = srv.response.header.frame_id;
    current_object_.object_location.target_pose.header.stamp = srv.response.header.stamp;
    current_object_.color = srv.response.color;
    current_object_.uid = srv.response.uid;

    current_object_.object_location.target_pose.pose.position.x = srv.response.point.x;
    current_object_.object_location.target_pose.pose.position.y = srv.response.point.y;
    current_object_.object_location.target_pose.pose.position.z = srv.response.point.z;

    state_ = StateEnum::kCommandPreparationGoal;
  } else {
    if (uid == -1)
      ROS_WARN("[Navigation Goals]: No object with color %d received from object map, going to State Waiting",
               srv.request.color);
    else
      ROS_WARN("[Navigation Goals]: No object with color %d and UID %d received from object map, going to State "
               "Waiting",
               srv.request.color, srv.request.uid);
    state_ = StateEnum::kWaiting;
    if (color == 400)
      ROS_WARN("[Navigation Goals]: Did you forget to select a color?");
  }
}

/**
 * @brief gets the position of an object from the object map
 *
 * @param color color of the object
 *
 * @return void
 */
void HelmoroNavigationGoals::GetObjectPosition(unsigned int color)
{
  GetObjectPosition(color, -1);
}

// Callbacks
/**
 * @brief Callback for joystick commands
 *
 * @param msg joystick message
 *
 * @return void
 */
void HelmoroNavigationGoals::joystickCallback(const sensor_msgs::JoyConstPtr& msg)
{
  start_collecting_objects_command_ = (msg->buttons[7] == 1);
  cancel_collecting_objects_command_ = (msg->buttons[6] == 1);

  if (msg->buttons[2]) {
    ROS_INFO("[Navigation Goals]: Selected to \n ***** Collect BLUE objects *****");
    selected_color_ = 115;
  } else if (msg->buttons[3]) {
    ROS_INFO("[Navigation Goals]: Selected to \n ***** Collect ORANGE objects *****");
    selected_color_ = 11;
  } else if (msg->buttons[0]) {
    ROS_WARN("[Navigation Goals]: Selected to \n ***** Collect GREEN objects, this is not tested *****");
    selected_color_ = 60;
  } else if (msg->buttons[1]) {
    ROS_INFO("[Navigation Goals]: Selected to \n ***** Collect RED objects *****");
    selected_color_ = 0;
  }
}

}  // namespace helmoro_navigation_goals
