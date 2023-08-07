#include "helmoro_motor_commands/helmoro_motor_commands.hpp"

#define CONSTRAIN(VAL, MIN_VAL, MAX_VAL) (VAL < MAX_VAL ? ((VAL > MIN_VAL ? VAL : MIN_VAL)) : MAX_VAL)

namespace helmoro_motor_commands
{
// 115200
HelmoroMotorCommands::HelmoroMotorCommands(NodeHandlePtr nh)
  : any_node::Node(nh), motors_left_("/dev/ttyACM1", 115200), motors_right_("/dev/ttyACM0", 115200)
{
}

HelmoroMotorCommands::~HelmoroMotorCommands()
{
}

bool HelmoroMotorCommands::init()
{
  // read parameters
  GetParams();

  // initialize pubs and subs
  move_base_sub_ = getNodeHandle().subscribe("/cmd_vel", 1, &HelmoroMotorCommands::CmdVelocityCallback, this);
  joint_state_pub_ = advertise<sensor_msgs::JointState>("joint_states_publisher", "/helmoro_joint_states", 1);
  imu_sub_ = getNodeHandle().subscribe("/imu/data", 1, &HelmoroMotorCommands::imuDataCallback, this);

  if (!is_real_robot_) {
    motor_commands_pub_ =
        advertise<helmoro_msgs::HelmoroJointCommandsShort>("motor_commands_publisher", "/helmoro_motor_commands", 1);
    motor_states_sub_ =
        getNodeHandle().subscribe("/helmoro_motor_states", 1, &HelmoroMotorCommands::motorStatesCallback, this);
  }

  // cmd_vel commands
  vx_cmd_ = 0.0;
  dtheta_cmd_ = 0.0;
  ros::Duration(1.0).sleep();

  wheel_vel_cmd_[0] = 0.0;
  wheel_vel_cmd_[1] = 0.0;
  wheel_vel_cmd_[2] = 0.0;
  wheel_vel_cmd_[3] = 0.0;

  omega_imu_ = 0.0;

  // publisher counter
  pub_counter_ = 0;

  // define rate of update loop
  auto workerTimeStep = param<double>("time_step_pub", 0.02);
  constexpr int priority = 0;
  addWorker("HelmoroMotorCommands::updateWorker", workerTimeStep, &HelmoroMotorCommands::update, this, priority);
  return true;
}

void HelmoroMotorCommands::cleanup()
{
  if (is_real_robot_) {
    motors_left_.SendDutyM1M2Command(addr_left_, 0, 0);
    motors_right_.SendDutyM1M2Command(addr_right_, 0, 0);
  }
}

bool HelmoroMotorCommands::update(const any_worker::WorkerEvent& event)
{
  std::unique_lock<std::mutex> lock(m_lock_);
  getWheelVelocitiesCommand();
  lock.unlock();
  if (is_real_robot_) {
    // read voltages
    // motors_right_.GetMainBatteryVoltage(addr_right_, batt_volt_right_);

    // read wheel positions
    motors_left_.GetM1M2Pos(addr_left_, wheel_pos_[1], wheel_pos_[3]);
    motors_right_.GetM1M2Pos(addr_right_, wheel_pos_[0], wheel_pos_[2]);

    wheel_pos_[0] = wheel_pos_[0] / tics_per_rad_;
    wheel_pos_[1] = wheel_pos_[1] / tics_per_rad_;
    wheel_pos_[1] = wheel_pos_[2] / tics_per_rad_;
    wheel_pos_[3] = wheel_pos_[3] / tics_per_rad_;

    // read wheel speeds
    motors_left_.GetM1SpeedFiltered(addr_left_, wheel_vel_state_[1]);
    motors_left_.GetM2SpeedFiltered(addr_left_, wheel_vel_state_[3]);
    motors_right_.GetM1SpeedFiltered(addr_right_, wheel_vel_state_[0]);
    motors_right_.GetM2SpeedFiltered(addr_right_, wheel_vel_state_[2]);

    wheel_vel_state_[0] = wheel_vel_state_[0] / tics_per_rad_;
    wheel_vel_state_[1] = wheel_vel_state_[1] / tics_per_rad_;
    wheel_vel_state_[2] = wheel_vel_state_[2] / tics_per_rad_;
    wheel_vel_state_[3] = wheel_vel_state_[3] / tics_per_rad_;

    // send wheel commands
    if (!motors_left_.SendSpeedM1M2Command(addr_left_, wheel_vel_cmd_[1] * tics_per_rad_,
                                           wheel_vel_cmd_[3] * tics_per_rad_))
      std::printf("Could not sent speed to left motors \n");
    if (!motors_right_.SendSpeedM1M2Command(addr_right_, wheel_vel_cmd_[0] * tics_per_rad_,
                                            wheel_vel_cmd_[2] * tics_per_rad_))
      std::printf("Could not send speed to right motors \n");
  } else {
    PublishMotorCommands();
  }

  // publish joint states
  PublishJointStates();

  return true;
}

void HelmoroMotorCommands::CmdVelocityCallback(const geometry_msgs::TwistConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(m_lock_);
  vx_cmd_ = msg->linear.x;
  dtheta_cmd_ = msg->angular.z;
}

void HelmoroMotorCommands::imuDataCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
  omega_imu_ = imu_msg->angular_velocity.z;
}

void HelmoroMotorCommands::motorStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  for (int i = 0; i < nb_actuators_; i++) {
    wheel_pos_[i] = msg->position[i];
    wheel_vel_state_[i] = msg->velocity[i];
  }
}

void HelmoroMotorCommands::GetParams()
{
  // Helmoro specs
  if (getNodeHandle().hasParam("/use_sim_time")) {
    is_real_robot_ = false;
    ROS_INFO("[Motor commands]: Commanding in simulation");
  }
  dx_wheels_ = param<double>("/helmoro_description/dimensions/wheel_spacing_x", 5.0);
  dy_wheels_ = param<double>("/helmoro_description/dimensions/wheel_spacing_y", 5.0);
  dia_wheels_ = param<double>("/helmoro_description/dimensions/wheel_diameter", 5.0);
  max_wheel_rot_vel_ = param<double>("/helmoro_description/actuators/max_wheel_rot_vel", 14.0);
  max_ang_vel_ = param<double>("/helmoro_description/velocities/max_ang_vel", 6.3);
  max_lin_vel_ = param<double>("/helmoro_description/velocities/max_lin_vel", 1);
  ;

  // Motor controller ports, addresses & baudrate
  addr_left_ = param<int>("motor_controllers/address_left", 128);
  addr_right_ = param<int>("motor_controllers/address_right", 129);
  baud_ = param<int>("motor_controllers/baudrate", 115200);
  tics_per_rad_ = param<int>("/helmoro_description/actuators/encoder_resolution", 2797) / (2 * M_PI);

  // integral factor
  ki_ = param<double>("integral/ki_factor", 1.0);
}

void HelmoroMotorCommands::getWheelVelocitiesCommand()
{
  // max rot speed for distinct radius
  double omega_max;
  double curve_radius;
  double linear_scaling_factor;
  double velocity_summ;
  double rot_lin_vel;
  double vel_sum_left;
  double vel_sum_right;

  // calculate wheel speed from base rotation
  rot_lin_vel = dtheta_cmd_ * (dy_wheels_ / 2);

  // calculate scaling factor
  if (vx_cmd_ + rot_lin_vel > max_lin_vel_) {
    linear_scaling_factor = max_lin_vel_ / (vx_cmd_ + rot_lin_vel);
  } else {
    linear_scaling_factor = 1;
  }

  // integration term on angular velocity error
  double dt = CONSTRAIN(timer_.ElapsedSeconds(), 0.0, 0.1);
  timer_.Reset();
  static double v_i = 0.0;

  if (vx_cmd_ + rot_lin_vel != 0) {
    v_i += ki_ * (dtheta_cmd_ - omega_imu_) * dt;
    v_i = CONSTRAIN(v_i, -0.15, 0.15);

    rot_lin_vel = rot_lin_vel + v_i;
  }

  vel_sum_left = (vx_cmd_ + rot_lin_vel) * linear_scaling_factor;
  vel_sum_right = (vx_cmd_ - rot_lin_vel) * linear_scaling_factor;

  wheel_vel_cmd_[0] = 2.0 * vel_sum_left / dia_wheels_;
  wheel_vel_cmd_[1] = 2.0 * vel_sum_right / dia_wheels_;
  wheel_vel_cmd_[2] = wheel_vel_cmd_[0];
  wheel_vel_cmd_[3] = wheel_vel_cmd_[1];
}

void HelmoroMotorCommands::PublishJointStates()
{
  sensor_msgs::JointState joint_states;
  joint_states.header.seq = pub_counter_;
  joint_states.header.stamp = ros::Time::now();
  for (int i = 0; i < nb_actuators_; i++) {
    joint_states.name.push_back(helmoro_description::HelmoroJointNames::getName(i));
    joint_states.position.push_back(wheel_pos_[i]);
    joint_states.velocity.push_back(wheel_vel_state_[i]);
  }

  joint_state_pub_.publish(joint_states);
  pub_counter_++;
}

void HelmoroMotorCommands::PublishMotorCommands()
{
  helmoro_msgs::HelmoroJointCommandsShort motor_commands;
  motor_commands.mode = static_cast<int>(helmoro_description::ActuatorModeEnum::MODE_JOINTVELOCITY);
  for (int i = 0; i < nb_actuators_; i++) {
    motor_commands.command[i] = wheel_vel_cmd_[i];
  }
  motor_commands_pub_.publish(motor_commands);
}

}  // namespace helmoro_motor_commands
