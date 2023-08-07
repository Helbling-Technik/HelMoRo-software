#pragma once

// Gazebo includes
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Message includes
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "helmoro_msgs/HelmoroWheelEncoderData.h"
#include "helmoro_msgs/HelmoroActuatorCommands.h"
#include "helmoro_description/enums/enums.hpp"
#include "helmoro_description/helmoro_names.hpp"
#include "helmoro_msgs/HelmoroJointCommands.h"
#include "helmoro_msgs/HelmoroJointCommandsShort.h"

// System includes
#include <boost/bind.hpp>
#include <stdio.h>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

namespace gazebo {

class Model_Helmoro : public ModelPlugin {

 public:
    /// Constructor for Gazebo plugin
    Model_Helmoro();

    /// Destructor
    ~Model_Helmoro();

    void init();

    /// The load function which is called when opening Gazebo
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Called by the world update start event
    void Update(const common::UpdateInfo & /*_info*/);

    /// Inits the ROS publishers
    void initPublishers();

    /// Inits the ROS subscribers
    void initSubscribers();

    /// Inits the ROS services
    void initServices();

    /// Reads parameters from the parameter server
    void readParameters();

    /// Reads Data from Simulation
    void readSimulation();

    // Writes Actuator Commands to simulation
    void writeSimulation();

    /// Publishes robot state over ROS
    void publishMotorStates();

    /// Callback for joint commands 
    void motorCommands_callback(const helmoro_msgs::HelmoroJointCommandsShortConstPtr& msg);

 private:
    static constexpr unsigned int noActuators_ = static_cast<int>(helmoro_description::ActuatorEnum::NrActuators);

    /* Gazebo */
    //! Pointer to the model
    physics::ModelPtr model_;
    //! Pointer to the update event connection
    event::ConnectionPtr updateConnection_;
    //! Pointer to sdf
    sdf::ElementPtr sdf_;

    /* ROS */
    //! ROS nodehandle
    ros::NodeHandle *nh_;

    //! ROS spinner
    ros::AsyncSpinner *spinner_;

    //! ROS publishers
    ros::Publisher motorStatesPub_;

    //! ROS subscribers
    ros::Subscriber motorCommandsSub_;

    //! ROS services

    /* params */
    bool publishState_;

    /* Execution Control and Timing */
    //! Save time stamp of last iteration
    common::Time lastTime_;
    //! Time step for PID controllers
    double summedDt_;
    double dt_;
    //! Loop rate for message sending
    double loopRate_;
    //! Sequence number for ROS messages
    unsigned int counter_;
    bool isPublishing_;

    /* States */
    //! Joint Torques to apply to simulation
    double tau_[noActuators_];
    //! Joint Forces read from simulation
    double motor_torques_[noActuators_];
    //! Joint Position read from simulation
    double motor_positions_[noActuators_];
    //! Joint Velocities read from simulation
    double motor_velocities_[noActuators_];

    double motor_commands_[noActuators_];
    int motor_commands_mode_;


    //! mutex locks (multiple threads)
    std::mutex mutexActuatorCommandsCallback_;
    std::mutex mutexControllerDefinition_;
    std::mutex mutexLoad_;
    std::mutex mutexOnUpdate_;
};

GZ_REGISTER_MODEL_PLUGIN(Model_Helmoro)
}
