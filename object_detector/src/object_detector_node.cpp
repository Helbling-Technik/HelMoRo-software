#include "object_detector/object_detector.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_detector");

  if (ros::master::check()) {
    helmoro::ObjectDetector object_detector;
    ros::spin();

    return 0;
  } else {
    std::printf("ROS core not running.\n");
    return -1;
  }
}


/*#include "any_node/Nodewrap.hpp"

int main(int argc, char **argv)
{  
  any_node::Nodewrap<helmoro::ObjectDetector> node(argc, argv, "object_detector", 1);// use 1 spinner thread
  node.execute();
  return 0;
}*/