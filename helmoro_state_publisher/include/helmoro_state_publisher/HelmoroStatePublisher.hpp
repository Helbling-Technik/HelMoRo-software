#pragma once

// message logger
#include "message_logger/message_logger.hpp"

// any_node
#include "any_node/Node.hpp"

#include <ros/ros.h>
#include "math.h"
#include <cmath>

#include <tf/transform_broadcaster.h>
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Imu.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_kdl/tf2_kdl.h>
#include <kdl/tree.hpp>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_broadcaster.h>
#include "helmoro_msgs/HelmoroWheelEncoderData.h"
#include "helmoro_description/enums/enums.hpp"
#include "helmoro_description/helmoro_names.hpp"
#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


namespace helmoro_state_publisher {

class HelmoroStatePublisher : public any_node::Node
{
    public:

        HelmoroStatePublisher(NodeHandlePtr nh);
        virtual ~HelmoroStatePublisher();
        /*HelmoroStatePublisher() = delete;  // constructor needs to take a shared_ptr to a ros::Nodehandle instance.
        explicit HelmoroStatePublisher(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh) {}

        ~HelmoroStatePublisher() override = default;
        */
        virtual bool init();
        virtual void cleanup();
        virtual bool update(const any_worker::WorkerEvent& event);

        void initPublishers();
        void initSubscribers();
        void readParameters();
        void getTransformFromKDL(KDL::Tree tree, std::string parent, std::string child, tf2::Quaternion& tf_quat, tf2::Matrix3x3& tf_rotm);
        void publishOdometry();
        void jointStateSubCallback_(const sensor_msgs::JointStateConstPtr& msg);
        void imuSubCallback_(const sensor_msgs::ImuConstPtr& imu_msg);

    private:
        //! ROS publishers
        ros::Publisher odomPub_;

        // ROS subscribers
        ros::Subscriber jointStateSub_;
        ros::Subscriber imuSub_;

        static constexpr unsigned int noActuators_ = static_cast<int>(helmoro_description::ActuatorEnum::NrActuators);

        ros::Time time_last_;        
        double wheel_radius_;
        double dx_;
        double dy_;
        std::map<std::string, double> joint_positions_;
        double x_dot_;
        double theta_dot_;
        //geometry_msgs::TransformStamped odom_tf_;
        geometry_msgs::Quaternion orientation_;
        //geometry_msgs::TransformStamped tf_imu_base_;
        tf2::Quaternion quat_imu_base_;
        tf2::Matrix3x3 rotm_imu_base_;
        bool firstImuMsg_;
        int imucounter_;
        tf2::Quaternion quat_reset_imu_;
        double odom_position_[3];
        geometry_msgs::Quaternion odom_quat;
        double theta_;
        //tf::TransformBroadcaster odom_broadcaster_;
        KDL::Tree helmoroKdlTree_;
        robot_state_publisher::RobotStatePublisher* statePublisherPtr_;
        tf::TransformBroadcaster odom_broadcaster_;

};

} // namespace helmoro_state_publisher