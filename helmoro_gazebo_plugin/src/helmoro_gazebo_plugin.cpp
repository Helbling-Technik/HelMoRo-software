#include "helmoro_gazebo_plugin/helmoro_gazebo_plugin.hpp"
#include <ros/package.h>
#include <Eigen/Core>


namespace gazebo {

Model_Helmoro::Model_Helmoro() : ModelPlugin(),
    nh_(),
    spinner_(),
    publishState_(true),
    isPublishing_(false),
    counter_(),
    lastTime_(),
    summedDt_(),
    loopRate_(),
    dt_()
{
}

Model_Helmoro::~Model_Helmoro(){

    delete spinner_;
    delete nh_;
}

void Model_Helmoro::init(){

  // Reset timers and Counters
  summedDt_ = 0;
  dt_ = 0;
  for(int i = 0; i < noActuators_; i++){
    motor_positions_[i] = 0.0;
    motor_velocities_[i] = 0.0;
    motor_torques_[i] = 0.0;
    tau_[i] = 0.0;
    motor_commands_[i] = 0.0;
  }
  
  lastTime_ = model_->GetWorld()->SimTime();

  readParameters();
  initPublishers();
  initSubscribers();
  initServices();

}

void Model_Helmoro::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

  std::lock_guard<std::mutex> lock(mutexLoad_);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()){

      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
  }

  // Store the pointer to the model
    model_ = _parent;
    sdf_ = _sdf;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Model_Helmoro::Update, this, _1));

  nh_ = new ros::NodeHandle("~");

  // Call init function
  this->init();


  for(unsigned int i=0; i<noActuators_; i++) {
      /// Apply joint torques
      model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i))->SetPosition(0, 0);
  }

  physics::PhysicsEnginePtr physics = model_->GetWorld()->Physics();
  const std::string frictionModel = "cone_model";
  physics->SetParam("friction_model", frictionModel);

}

// Called by the world update start event
void Model_Helmoro::Update(const common::UpdateInfo & /*_info*/){

    std::lock_guard<std::mutex> lock(mutexOnUpdate_);

    // read from simulation
    readSimulation();

    common::Time curTime = model_->GetWorld()->SimTime();
    dt_ = (curTime - lastTime_).Double();
    lastTime_ = curTime;
    
    summedDt_ += dt_;

    if(summedDt_ >= 1.0/loopRate_){

        isPublishing_ = true;

        // publish msgs
        publishMotorStates();

        // increase message sequence counter
        counter_++;
        summedDt_ -= 1.0/double(loopRate_);
    }
    else{
        isPublishing_ = false;
    }

    // publish new joint states/forces and receive new force references
    ros::spinOnce();

    {
      std::lock_guard<std::mutex> lock(mutexActuatorCommandsCallback_);
      std::lock_guard<std::mutex> lock2(mutexControllerDefinition_);

      // update controller and apply control signal to joints. references are updated in the callback
      writeSimulation();
    }

}

void Model_Helmoro::initPublishers(){

    motorStatesPub_ = nh_->advertise<sensor_msgs::JointState>("/helmoro_motor_states", 10, this);
}

void Model_Helmoro::initSubscribers(){

    // actuator commands
    motorCommandsSub_ = nh_->subscribe("/helmoro_motor_commands", 1, &Model_Helmoro::motorCommands_callback, this);
}

void Model_Helmoro::initServices(){

    // init ros service client
}

void Model_Helmoro::readParameters(){

  loopRate_ = 100.0;
}

void Model_Helmoro::readSimulation(){

  //! Read joint states
  // get force, position and velocity of each joint
  for(unsigned int i=0; i<noActuators_; i++) {

      auto joint = model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i));
      if (!joint) {

          ROS_ERROR_STREAM_NAMED("gazebo_ros_control", "Joint " << helmoro_description::HelmoroJointNames::getName(i) << " does not exist in Gazebo.");
          return;
      }
      motor_torques_[i] = joint->GetForce(i);
      motor_positions_[i] = joint->Position(i);
      motor_velocities_[i] = joint->GetVelocity(i);
  }
}

void Model_Helmoro::writeSimulation(){

  /// calculate controller outputs
  for(unsigned int i=0; i<noActuators_; i++) {
      model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i))->SetVelocity(0, motor_commands_[i]);
      /*switch(joint_commands_mode_){
        case static_cast<int>(helmoro_description::ActuatorModeEnum::MODE_SHUTOFF):
          model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i))->SetForce(0, 0);
        case static_cast<int>(helmoro_description::ActuatorModeEnum::MODE_JOINTPOSITION):
          model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i))->SetPosition(0, joint_commands_[i]);
        case static_cast<int>(helmoro_description::ActuatorModeEnum::MODE_JOINTVELOCITY):
          model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i))->SetVelocity(0, joint_commands_[i]);
        case static_cast<int>(helmoro_description::ActuatorModeEnum::MODE_JOINTTORQUE):
          model_->GetJoint(helmoro_description::HelmoroJointNames::getName(i))->SetForce(0, joint_commands_[i]);
      }*/
  }
}

void Model_Helmoro::publishMotorStates() {

    if (!publishState_) return;
    ///if (fullJointStatesPub_.getNumSubscribers() == 0) return;

    sensor_msgs::JointState motorStates;
    motorStates.header.stamp = ros::Time(lastTime_.sec, lastTime_.nsec);
    motorStates.header.seq = counter_;
    motorStates.name.resize(noActuators_);
    motorStates.position.resize(noActuators_);
    motorStates.velocity.resize(noActuators_);
    for (unsigned int i=0; i<noActuators_; i++) {

        motorStates.name[i] = helmoro_description::HelmoroJointNames::getName(i);
        motorStates.position[i] = motor_positions_[i];
        motorStates.velocity[i] = motor_velocities_[i];
    }
    
    motorStatesPub_.publish(motorStates);
}
 
void Model_Helmoro::motorCommands_callback(const helmoro_msgs::HelmoroJointCommandsShortConstPtr& msg){                                                                                  
  std::lock_guard<std::mutex> lock(mutexActuatorCommandsCallback_);
  for(int i = 0; i < noActuators_; i++) {
    motor_commands_mode_ = msg->mode;
    motor_commands_[i] = msg->command[i];
  }
}

}  // namespace gazebo
