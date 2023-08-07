#include "helmoro_state_publisher/HelmoroStatePublisher.hpp"

// nodewrap
#include "any_node/Nodewrap.hpp"


int main(int argc, char** argv)
{
  any_node::Nodewrap<helmoro_state_publisher::HelmoroStatePublisher> node(argc, argv, "HelmoroStatePublisher", 1);// use 2 spinner thread
  node.execute();
  return 0;
  //return static_cast<int>(!node.execute());  // execute blocks until the node was requested to shut down (after reception of a signal (e.g.
  // SIGINT) or after calling the any_node::Node::shutdown() function)
}