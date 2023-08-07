#pragma once
// motor controller
#include "helmoro_motor_commands/motor_interface/motor_interface.h"
#include "helmoro_motor_commands/timer/timer.h"

// message logger
#include "message_logger/message_logger.hpp"

// any_node
#include "any_node/Node.hpp"

// STD
#include <math.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Helmoro
#include "helmoro_description/enums/enums.hpp"
#include "helmoro_description/helmoro_names.hpp"
#include "helmoro_msgs/HelmoroJointCommandsShort.h"

// ros
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

namespace helmoro_motor_commands
{
class HelmoroMotorCommands : public any_node::Node
{
public:
  HelmoroMotorCommands(NodeHandlePtr nh);
  virtual ~HelmoroMotorCommands();

  virtual bool init();
  virtual void cleanup();
  virtual bool update(const any_worker::WorkerEvent& event);

  void CmdVelocityCallback(const geometry_msgs::TwistConstPtr& msg);
  void imuDataCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void motorStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  void getWheelVelocitiesCommand();
  void GetParams();
  void PublishJointStates();
  void PublishMotorCommands();

private:
  ros::Publisher joint_state_pub_;
  ros::Publisher motor_commands_pub_;  // only used for gazebo simulation
  ros::Subscriber motor_states_sub_;   // only used for gazebo simulation
  ros::Subscriber move_base_sub_;
  ros::Subscriber imu_sub_;

  static constexpr unsigned int nb_actuators_ = static_cast<int>(helmoro_description::ActuatorEnum::NrActuators);
  bool is_real_robot_{true};

  // motor controllers
  std::string port_left_;
  std::string port_right_;
  uint8_t addr_left_;
  uint8_t addr_right_;
  int baud_;
  MotorInterface motors_left_;
  MotorInterface motors_right_;

  // encoder

  double tics_per_rad_;

  // battery voltages
  double batt_volt_left_;
  double batt_volt_right_;

  // wheel speeds
  double wheel_vel_state_[nb_actuators_] = {0.0, 0.0, 0.0, 0.0};
  double wheel_vel_filtered_[nb_actuators_] = {0.0, 0.0, 0.0, 0.0};
  double wheel_pos_[nb_actuators_];

  // move_base
  double dx_wheels_;
  double dy_wheels_;
  double dia_wheels_;
  double max_wheel_rot_vel_;
  double max_ang_vel_;
  double max_lin_vel_;
  double wheel_vel_cmd_[nb_actuators_] = {0.0, 0.0, 0.0, 0.0};
  double vx_cmd_;
  double dtheta_cmd_;

  // publish joint states
  int pub_counter_;

  // IMU Integral
  double omega_imu_;
  double ki_ = 1.0;
  Timer timer_;

  // mutex lock
  std::mutex m_lock_;
};

}  // namespace helmoro_motor_commands
