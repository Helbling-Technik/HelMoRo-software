#ifndef SIMPLE_LAYER_H_
#define SIMPLE_LAYER_H_
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <vector>

#include "geometry_msgs/Point.h"
#include "ros/subscriber.h"
#include "visualization_msgs/MarkerArray.h"

namespace object_map_layer_namespace
{
class ObjectMapLayer : public costmap_2d::Layer
{
public:
  ObjectMapLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  void matchSize();

  /** @brief  Given a distance, compute a cost.
   * @param  distance The distance from an obstacle in cells
   * @return A cost value for the distance */
  inline double computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0)
      cost = costmap_2d::LETHAL_OBSTACLE;
    else if (distance * resolution_ <= inscribed_radius_)
      cost = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    else if (distance * resolution_ > inflation_radius_)
      cost = 0;
    else {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = ((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }

private:
  void reconfigureCB(costmap_2d::InflationPluginConfig& config, uint32_t level);

  dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>* dsrv_;

  std::vector<geometry_msgs::Point> object_2d_list;
  ros::Subscriber object_map_sub_;
  bool map_received_{false};

  // Params
  double inscribed_radius_, weight_, resolution_;
  double inflation_radius_, cell_inflation_radius_;

  // Callback
  void objectMapMarkerCallback(const visualization_msgs::MarkerArray& msg);
};
}  // namespace object_map_layer_namespace
#endif