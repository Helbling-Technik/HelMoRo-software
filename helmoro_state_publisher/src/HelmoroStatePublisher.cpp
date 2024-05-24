#include "helmoro_state_publisher/HelmoroStatePublisher.hpp"

/**
 * @file HelmoroStatePublisher.cpp
 * @brief Implementation file for the HelmoroStatePublisher class.
 * 
 * This file contains the implementation of the HelmoroStatePublisher class, which is responsible for publishing the state of the HelMoRo robot.
 * The class initializes publishers and subscribers, reads parameters, and updates the robot state.
 * The class also publishes the robot's odometry as a nav_msg and tf.
 */

namespace helmoro_state_publisher {

/**
 * @brief Constructor for the HelmoroStatePublisher class.
 * 
 * @param nh A shared pointer to the ROS node handle.
 */
HelmoroStatePublisher::HelmoroStatePublisher(NodeHandlePtr nh):
    any_node::Node(nh),
    odomPub_()
{
}

/**
 * @brief Destructor for the HelmoroStatePublisher class.
 */
HelmoroStatePublisher::~HelmoroStatePublisher(){

}

/**
 * @brief Initializes the HelmoroStatePublisher class.
 * 
 * @return True if initialization was successful, false otherwise.
 */
bool HelmoroStatePublisher::init() {

    time_last_ = ros::Time::now();
    odom_position_ [0] = 0.0;
    odom_position_ [1] = 0.0;
    odom_position_ [2] = 0.06;
    orientation_.x = 0;
    orientation_.y = 0;
    orientation_.z = 0;
    orientation_.w = 1;
    theta_ = 0.0;
    firstImuMsg_ = true;
    imucounter_ = 0;

    readParameters();
    getTransformFromKDL(helmoroKdlTree_, "imu_link", "base_link", quat_imu_base_, rotm_imu_base_);
    initPublishers();
    initSubscribers();

    statePublisherPtr_ = new robot_state_publisher::RobotStatePublisher(helmoroKdlTree_);
    statePublisherPtr_->publishFixedTransforms("", true);

    auto workerTimeStep = param<double>("time_step_pub", 0.02);
    constexpr int priority = 0;
    addWorker("HelmoroStatePublisher::updateWorker", workerTimeStep, &HelmoroStatePublisher::update, this, priority);
    return true;
}

/**
 * @brief Cleans up the HelmoroStatePublisher class.
 */
void HelmoroStatePublisher::cleanup() {

}

/**
 * @brief Updates the robot state.
 * 
 * @param event The worker event.
 * @return True if the update was successful, false otherwise.
 */
bool HelmoroStatePublisher::update(const any_worker::WorkerEvent& event) {

    publishOdometry();
    statePublisherPtr_->publishTransforms(joint_positions_, ros::Time::now(), "");
    return true;
}

/**
 * @brief Reads the parameters for the HelmoroStatePublisher class.
 */
void HelmoroStatePublisher::readParameters(){

    dx_ = param<double>("/helmoro_description/dimensions/wheel_spacing_x", 5.0);
    dy_ = param<double>("/helmoro_description/dimensions/wheel_spacing_y", 5.0);
    wheel_radius_ = param<double>("/helmoro_description/dimensions/wheel_diameter", 5.0)/2.0;

    std::string robot_desc_string;
    robot_desc_string = param<std::string>("/helmoro_description/urdf", "");
    if (!kdl_parser::treeFromString(robot_desc_string, helmoroKdlTree_)){
        ROS_ERROR("Failed to construct kdl tree");
    }
}

/**
 * @brief Gets the transform from KDL.
 * 
 * @param tree The KDL tree.
 * @param parent The parent frame.
 * @param child The child frame.
 * @param tf_quat The quaternion.
 * @param tf_rotm The rotation matrix.
 */
void HelmoroStatePublisher::getTransformFromKDL(KDL::Tree tree, std::string parent, std::string child, tf2::Quaternion& tf_quat, tf2::Matrix3x3& tf_rotm){

    KDL::Chain chain;
    tree.getChain(parent, child, chain);
    KDL::Frame frame = chain.getSegment(0).getFrameToTip();
    geometry_msgs::TransformStamped tf_transform;
    tf_transform = tf2::kdlToTransform(frame);
    tf2::fromMsg(tf_transform.transform.rotation, tf_quat);
    tf_rotm.setRotation(tf_quat);
}

/**
 * @brief Initializes the publishers for the HelmoroStatePublisher class.
 */
void HelmoroStatePublisher::initPublishers(){
    odomPub_ = getNodeHandle().advertise<nav_msgs::Odometry>("/odom", 1, this);
    //odomPub_ = advertise<nav_msgs::Odometry>("odometry_publisher", "/odom", 1);
}

/**
 * @brief Initializes the subscribers for the HelmoroStatePublisher class.
 */
void HelmoroStatePublisher::initSubscribers(){

    jointStateSub_ = getNodeHandle().subscribe("/helmoro_joint_states", 1,  &HelmoroStatePublisher::jointStateSubCallback_, this);
    imuSub_ = getNodeHandle().subscribe("/imu/data", 1, &HelmoroStatePublisher::imuSubCallback_, this);
}

/**
 * @brief Publishes the robot's odometry as a nav_msg and tf.
 */
void HelmoroStatePublisher::publishOdometry() {

    // Publish odom as nav_msg
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = helmoro_description::HelmoroLinkNames::getName(0);
    odom.pose.pose.position.x = odom_position_[0];
    odom.pose.pose.position.y = odom_position_[1];
    odom.pose.pose.position.z = odom_position_[2];
    //odom.pose.pose.orientation = orientation_;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta_);
    odom.twist.twist.linear.x = x_dot_;
    odom.twist.twist.angular.z = theta_dot_;

    odomPub_.publish(odom);

    // Publish odom as tf
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = helmoro_description::HelmoroLinkNames::getName(0);
    odom_tf.transform.translation.x = odom_position_[0];
    odom_tf.transform.translation.y = odom_position_[1];
    odom_tf.transform.translation.z = odom_position_[2];
    //odom_tf.transform.rotation = orientation_;
    odom_tf.transform.rotation = tf::createQuaternionMsgFromYaw(theta_);
    
    odom_broadcaster_.sendTransform(odom_tf);
    
}

/**
 * @brief Callback function for the joint state subscriber.
 * 
 * @param msg The joint state message.
 */
void HelmoroStatePublisher::jointStateSubCallback_(const sensor_msgs::JointStateConstPtr& msg){

    ros::Time time_current;
    time_current = msg->header.stamp;
    double dt = time_current.toSec() - time_last_.toSec();
    
    //std::map<std::string, double> joint_positions;
    double joint_velocities[4];
    for (unsigned int i=0; i<msg->name.size(); i++) {
      joint_positions_.insert(make_pair(msg->name[i], msg->position[i]));
      joint_velocities[i] = msg->velocity[i];
    }
    //jointStateHeader_ = msg->header.stamp;

    x_dot_ = wheel_radius_*(joint_velocities[0] + joint_velocities[1] + joint_velocities[2] + joint_velocities[3])/4.0;
    odom_position_[0] += x_dot_*cos(theta_)*dt;
    odom_position_[1] += x_dot_*sin(theta_)*dt;
    //statePublisherPtr_->publishTransforms(joint_positions, msg->header.stamp, "");

    time_last_ = time_current;
}

/**
 * @brief Callback function for the IMU subscriber.
 * 
 * @param imu_msg The IMU message.
 */
void HelmoroStatePublisher::imuSubCallback_(const sensor_msgs::ImuConstPtr& imu_msg){
    
    if(!std::isnan(imu_msg->orientation.x)){
        if(firstImuMsg_){
            //if(imu_msg->orientation.x != 0){
            if(imucounter_ >= 20){
                tf2::Quaternion quatzero(0,0,0,1);
                tf2::Quaternion quatfirstimu;
                tf2::fromMsg(imu_msg->orientation, quatfirstimu);
                quatfirstimu[3] = -quatfirstimu[3]; //calculate inverse
                quat_reset_imu_ = quatzero * quatfirstimu;

                firstImuMsg_ = false;
            }
            imucounter_ = imucounter_ + 1; 
        }
        else{
            //theta_dot_ = imu_msg->angular_velocity.z;
            theta_dot_ = rotm_imu_base_[2][0]*imu_msg->angular_velocity.x + rotm_imu_base_[2][1]*imu_msg->angular_velocity.y + rotm_imu_base_[2][2]*imu_msg->angular_velocity.z;
            
            //orientation_ = imu_msg->orientation;
            tf2::Quaternion quat_tf, quat_tf_transformed;
            tf2::fromMsg(imu_msg->orientation, quat_tf);
            quat_tf_transformed = quat_imu_base_*quat_reset_imu_*quat_tf;
            quat_tf_transformed.normalize();
            orientation_ = tf2::toMsg(quat_tf_transformed);
            theta_ = tf::getYaw(orientation_);
        }
    }
}

} // namespace helmoro_state_publisher

