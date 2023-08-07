#include "object_detector/object_detector.hpp"

// helmoro
#include "helmoro_msgs/ObjectDetections.h"
#include "ros/time.h"
#include "sensor_msgs/image_encodings.h"

// STL
#include <algorithm>
#include <string>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace helmoro
{
// constructor
ObjectDetector::ObjectDetector(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : tf_buffer_(), sub_tf_(tf_buffer_)
{
  // general variables
  nh_ = nh;
  pnh_ = pnh;

  // parameters

  if (nh_.hasParam("/use_sim_time")) {
    is_simulation_ = true;
    depth_encoding_ = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_unit_conversion_factor_ = 1.0f;
  }

  pnh_.getParam("hue", hue_);
  int hue_width;
  pnh_.getParam("hue_width", hue_width);

  std::vector<int> sv_low, sv_high;
  pnh_.getParam("sv_low", sv_low);
  pnh_.getParam("sv_high", sv_high);

  for (int i = 0; i < hue_.size(); ++i) {
    int hue_low = hue_[i] - hue_width;
    while (hue_low < 0)
      hue_low += 180;

    int hue_high = (hue_[i] + hue_width) % 180;

    hsv_low_.push_back({hue_low, sv_low[0], sv_low[1]});
    hsv_high_.push_back({hue_high, sv_high[0], sv_high[1]});
  }

  pnh_.getParam("mean_block_half_width", mean_block_half_width_);

  pnh_.getParam("max_object_height", max_object_height_);

  pnh_.getParam("object_detection_valid_after", object_detection_valid_after_);
  object_detector_counter_ = object_detection_valid_after_;  // Setting for first time the object detection counter

  ros::Subscriber sub_cam_info =
      nh_.subscribe("/camera/rgb/camera_info", 1, &ObjectDetector::updateCameraParameters, this);
  do {
    ROS_INFO("[object_detector]: Waiting for camera parameters...");
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  } while (!cam_initialized_);
  sub_cam_info.shutdown();
  ROS_INFO("[object_detector]: done");

  image_transport::ImageTransport it(nh_), pit(pnh_);
  sub_image_ = it.subscribe("/camera/rgb/image_raw", 1, &ObjectDetector::imageCallback, this);
  sub_depth_ = it.subscribe("/camera/depth/image_raw", 1, &ObjectDetector::depthCallback, this);

  pub_object_detection_ = pit.advertise("object_detections_image", 1);
  pub_object_positions_ = pnh_.advertise<helmoro_msgs::ObjectDetections>("object_detections", 1);
}

ObjectDetector::~ObjectDetector(void)
{
}

// parameters
void ObjectDetector::updateCameraParameters(const sensor_msgs::CameraInfoConstPtr &msg)
{
  cam_intrinsics_.setIdentity();
  cam_intrinsics_(0, 0) = msg->K[0];
  cam_intrinsics_(0, 2) = msg->K[2];
  cam_intrinsics_(1, 1) = msg->K[4];
  cam_intrinsics_(1, 2) = msg->K[5];

  cam_intrinsics_inv_ = cam_intrinsics_.inverse();

  float focal_length = msg->P[0];

  // shift of camera center point and dividing by focal length results in vector in world coords
  camera_position_vector_ = vec3_t(-msg->P[3], -msg->P[7], -msg->P[11]) / focal_length;
  cam_initialized_ = true;
}

// callbacks
void ObjectDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  if (cv_ptr->image.empty())
    return;

  // convert color space
  cv::Mat hsv;
  cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

  // hsv color finder, uncomment if necessary
  /*cv::circle(cv_ptr->image, cv::Point2i(320,240), 10, cv::Scalar(0,0,0));
  std::printf("h: %d s: %d v: %d\n", hsv.at<cv::Vec3b>(240, 320)[0],
                                     hsv.at<cv::Vec3b>(240, 320)[1],
                                     hsv.at<cv::Vec3b>(240, 320)[2]);*/

  // iterate through all colors
  helmoro_msgs::ObjectDetections object_detections_msg;
  object_detections_msg.header = msg->header;
  object_detections_msg.header.frame_id = "map";
  object_detections_msg.num_detections_total = 0;
  for (int i = 0; i < hue_.size(); ++i) {
    // filter color
    cv::Mat mask;
    if (hsv_low_[i][0] < hsv_high_[i][0])  // Note: this check is not safe :)
    {
      cv::inRange(hsv, cv::Scalar(hsv_low_[i][0], hsv_low_[i][1], hsv_low_[i][2]),
                  cv::Scalar(hsv_high_[i][0], hsv_high_[i][1], hsv_high_[i][2]), mask);
    } else {
      cv::Mat mask0, mask1;
      cv::inRange(hsv, cv::Scalar(hsv_low_[i][0], hsv_low_[i][1], hsv_low_[i][2]),
                  cv::Scalar(180, hsv_high_[i][1], hsv_high_[i][2]), mask0);

      cv::inRange(hsv, cv::Scalar(0, hsv_low_[i][1], hsv_low_[i][2]),
                  cv::Scalar(hsv_high_[i][0], hsv_high_[i][1], hsv_high_[i][2]), mask1);

      mask = cv::max(mask0, mask1);
    }

    // filter mask
    cv::Mat kernel_3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat kernel_5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    for (int j = 0; j < 2; ++j) {
      cv::erode(mask, mask, kernel_3x3);
      cv::erode(mask, mask, kernel_3x3);
      cv::dilate(mask, mask, kernel_5x5);
    }

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int j = 0; j < contours.size(); j++) {
      float area = cv::contourArea(contours[j]);
      float min_radius;
      cv::Point2f center;
      cv::minEnclosingCircle(contours[j], center, min_radius);

      if (area > 250.f && area / (M_PI * min_radius * min_radius) > 0.6) {
        std::lock_guard<std::mutex> lck(depth_m_);

        cv::circle(cv_ptr->image, center, 2, cv::Scalar(0, 0, 0), 2);

        float mean_depth = 0.f;
        int count = 0;
        for (int x = std::max(0, (int)center.x - mean_block_half_width_);
             x < std::min((int)center.x + mean_block_half_width_, depth_.cols); ++x) {
          for (int y = std::max(0, (int)center.y - mean_block_half_width_);
               y < std::min((int)center.y + mean_block_half_width_, depth_.rows); ++y) {
            // As the simulation and real robot use a different encoding here it needs to be seperated. in C++17 it
            // would be possible to use .at<auto> but not in c+11
            if (is_simulation_) {
              if (depth_.at<float>(y, x) > 0) {
                // TODo verify that the depth works as well with real camera
                mean_depth += (float)(depth_.at<float>(y, x));
                ++count;
              }
            } else {
              if (depth_.at<ushort>(y, x) > 0) {
                // TODo verify that the depth works as well with real camera
                mean_depth += (float)(depth_.at<ushort>(y, x));
                ++count;
              }
            }
          }
        }

        if (count) {
          mean_depth /= (float)count;

          vec3_t center_pt = compute3DPoint(center, mean_depth * depth_unit_conversion_factor_);

          geometry_msgs::Point c_pt, m_pt;
          c_pt.x = center_pt(0);
          c_pt.y = center_pt(1);
          c_pt.z = center_pt(2);

          geometry_msgs::TransformStamped tf_stamped;
          try {
            tf_stamped = tf_buffer_.lookupTransform("map", "camera_rgb_optical_frame", ros::Time(0));
          } catch (tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            return;
          }
          tf2::doTransform(c_pt, m_pt, tf_stamped);

          if (m_pt.z < max_object_height_) {
            object_detector_counter_--;
            if (object_detector_counter_ == 0) {  // check if the object is seen long enough
              object_detections_msg.num_detections_total++;
              object_detections_msg.color.push_back(hue_[i]);
              object_detections_msg.point.push_back(m_pt);
              //ROS_INFO("[object_detector]: object in sight");
              //ROS_INFO("[object_detector]: object_pos in mapframe: \n X: %f , Y: %f, Z: %f", m_pt.x, m_pt.y, m_pt.z);
              //ROS_INFO("[object_detector]: number of detections %d", object_detections_msg.num_detections_total);
              object_detector_counter_ = object_detection_valid_after_;
            }
          } else {
            object_detector_counter_ = object_detection_valid_after_;
          }
        }
      }
    }
  }

  // if (pub_object_positions_.getNumSubscribers() > 0) pub_object_positions_.publish(object_detections_msg);
  pub_object_positions_.publish(object_detections_msg);

  if (pub_object_detection_.getNumSubscribers() > 0) {
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "bgr8";
    cv_image.image = cv_ptr->image;  // publishing Image with circle on detected object
    pub_object_detection_.publish(cv_image.toImageMsg());
  }
}

void ObjectDetector::depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, depth_encoding_);

  std::lock_guard<std::mutex> lck(depth_m_);
  cv_ptr->image.copyTo(depth_);
}

ObjectDetector::vec3_t ObjectDetector::compute3DPoint(cv::Point2f uv, float depth)
{
  return cam_intrinsics_inv_ * vec3_t(uv.x, uv.y, 1.0) * depth + camera_position_vector_;
}
}  // namespace helmoro
