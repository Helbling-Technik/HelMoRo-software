#include <pluginlib/class_list_macros.h>

#include <cmath>
#include <object_map/object_map_layer.hpp>

#include "geometry_msgs/Point.h"

PLUGINLIB_EXPORT_CLASS(object_map_layer_namespace::ObjectMapLayer, costmap_2d::Layer)

namespace object_map_layer_namespace
{
ObjectMapLayer::ObjectMapLayer() : inflation_radius_(0), weight_(0), cell_inflation_radius_(0), dsrv_(NULL)
{
}

void ObjectMapLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  object_map_sub_ = nh.subscribe("/object_map/object_map_markers", 1, &ObjectMapLayer::objectMapMarkerCallback, this);

  ros::Rate r(1);
  while (!map_received_ && nh.ok()) {
    ROS_INFO("[Object Map Layer]: Waiting for object_map");
    ros::spinOnce();
    r.sleep();
  }

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb =
      boost::bind(&ObjectMapLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  matchSize();
}

void ObjectMapLayer::reconfigureCB(costmap_2d::InflationPluginConfig& config, uint32_t level)
{
  enabled_ = config.enabled;
  weight_ = config.cost_scaling_factor;
  inflation_radius_ = config.inflation_radius;
}

void ObjectMapLayer::matchSize()
{
  // boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  inscribed_radius_ = layered_costmap_->getInscribedRadius();

  cell_inflation_radius_ = costmap->cellDistance(inflation_radius_);
}

void ObjectMapLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                  double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  // TODO UNSURE IF THIS IS NEEDED
  *min_x = std::min(*min_x, robot_x);
  *min_y = std::min(*min_y, robot_y);
  *max_x = std::max(*max_x, robot_x);
  *max_y = std::max(*max_y, robot_y);
}

void ObjectMapLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;

  unsigned int min_mx, min_my, max_mx, max_my;
  for (geometry_msgs::Point obstacle_point : object_2d_list) {
    if (master_grid.worldToMap(obstacle_point.x, obstacle_point.y, mx, my)) {
      master_grid.worldToMap(obstacle_point.x - inflation_radius_, obstacle_point.y - inflation_radius_, min_mx,
                             min_my);
      master_grid.worldToMap(obstacle_point.x + inflation_radius_, obstacle_point.y + inflation_radius_, max_mx,
                             max_my);
      for (double iter_x = min_mx; iter_x < max_mx; ++iter_x) {
        for (double iter_y = min_my; iter_y < max_my; ++iter_y) {
          double distance = std::hypot(std::abs(double((double)mx - iter_x)), std::abs(double((double)my - iter_y)));
          master_grid.setCost(iter_x, iter_y, computeCost(distance));
        }
      }
    }
  }
}

void ObjectMapLayer::objectMapMarkerCallback(const visualization_msgs::MarkerArray& msg)
{
  map_received_ = true;
  object_2d_list.clear();  // Clearing the vector when a new message arrives
  for (auto object : msg.markers) {
    geometry_msgs::Point p;
    p.x = object.pose.position.x;
    p.y = object.pose.position.y;
    p.z = 0;  // 2D map does not require a z-coordinate
    object_2d_list.push_back(p);
  }
}

}  // namespace object_map_layer_namespace