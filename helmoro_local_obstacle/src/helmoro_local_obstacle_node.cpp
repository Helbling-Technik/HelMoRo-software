#include "helmoro_local_obstacle/HelmoroLocalObstacle.hpp"

// nodewrap
#include "any_node/Nodewrap.hpp"

int main(int argc, char **argv)
{
  any_node::Nodewrap<helmoro_local_obstacle::HelmoroLocalObstacle> node(argc, argv, "helmoro_local_obstacle", 1);// use 1 spinner thread
  node.execute();
  return 0;
}
