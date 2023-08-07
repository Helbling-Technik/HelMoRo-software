#pragma once

// message logger
#include "message_logger/message_logger.hpp"

// any_node
#include "any_node/Node.hpp"

// ros
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/Joy.h>

// stl
#include <string>

namespace helmoro_joymanager
{
class HelmoroJoyManager : public any_node::Node
{
public:
  HelmoroJoyManager(NodeHandlePtr nh);
  virtual ~HelmoroJoyManager();

  virtual bool init();
  virtual void cleanup();
  virtual bool update(const any_worker::WorkerEvent& event);

  void joystickCallback(const sensor_msgs::JoyConstPtr& msg);

private:
  ros::Publisher userInputPublisher_;
  ros::Subscriber joystickSubscriber_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;

  double linearVelocityScalingFactor_;
  double angularVelocityScalingFactor_;
  double maxLinVel_;
  double maxAngVel_;
  bool right_trigger_initalized_;
  bool left_trigger_initalized_;
};

}  // namespace helmoro_joymanager
