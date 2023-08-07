#include "helmoro_joymanager/HelmoroJoyManager.hpp"

namespace helmoro_joymanager
{
HelmoroJoyManager::HelmoroJoyManager(NodeHandlePtr nh)
  : any_node::Node(nh)
  , action_client_("move_base")
  , userInputPublisher_()
  , linearVelocityScalingFactor_()
  , angularVelocityScalingFactor_()
{
}

HelmoroJoyManager::~HelmoroJoyManager()
{
}

bool HelmoroJoyManager::init()
{
  // initialize publishers and subscribers
  userInputPublisher_ = getNodeHandle().advertise<geometry_msgs::Twist>("/cmd_vel", 1, this);
  joystickSubscriber_ = getNodeHandle().subscribe("/helmoro_joystick", 1, &HelmoroJoyManager::joystickCallback, this);

  // load scaling factors from parameter file
  linearVelocityScalingFactor_ = param<double>("scaling_factors/linear_velocity", 1.0);
  angularVelocityScalingFactor_ = param<double>("scaling_factors/angular_velocity", 1.0);

  maxLinVel_ = param<double>("/helmoro_description/velocities/max_lin_vel", 1);
  maxAngVel_ = param<double>("/helmoro_description/velocities/max_ang_vel", 6.3);
  right_trigger_initalized_ = false;  // needs to be 0 so that velocity commands are accepted
  left_trigger_initalized_ = false;

  return true;
}

void HelmoroJoyManager::cleanup()
{
}

bool HelmoroJoyManager::update(const any_worker::WorkerEvent& event)
{
  return true;
}

void HelmoroJoyManager::joystickCallback(const sensor_msgs::JoyConstPtr& msg)
{
  geometry_msgs::Twist cmd_vel;

  // check if the right trigger has been initalized
  if ((not right_trigger_initalized_) && msg->axes[5] != 0)
    right_trigger_initalized_ = true;

  if ((not left_trigger_initalized_) && msg->axes[2] != 0)
    left_trigger_initalized_ = true;

  // set the linear and angular velocity fo the robot
  if (right_trigger_initalized_ && left_trigger_initalized_) {  // only if no value is == 0, as this is the case at INIT
    cmd_vel.linear.x = (-(msg->axes[5] - 1) - (-(msg->axes[2] - 1))) * 0.5 * linearVelocityScalingFactor_ * maxLinVel_;
  }

  if (msg->buttons[1]) {  // If the red button is pushed all goals are cancelled
    action_client_.cancelAllGoals();
    ROS_INFO("Cancelled all goals for Helmoro");
  }
  cmd_vel.angular.z = msg->axes[0] * angularVelocityScalingFactor_ * maxAngVel_;

  userInputPublisher_.publish(cmd_vel);
}

}  // namespace helmoro_joymanager
