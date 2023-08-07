#include "object_map/object_map.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_map");

  if (ros::master::check()) {
    helmoro::ObjectMap object_map;
    ros::spin();

    return 0;
  } else {
    std::printf("ROS core not running.\n");
    return -1;
  }
}