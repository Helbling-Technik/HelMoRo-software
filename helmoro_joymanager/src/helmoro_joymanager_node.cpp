#include "helmoro_joymanager/HelmoroJoyManager.hpp"

// nodewrap
#include "any_node/Nodewrap.hpp"

int main(int argc, char **argv)
{
  any_node::Nodewrap<helmoro_joymanager::HelmoroJoyManager> node(argc, argv, "helmoro_joymanager",
                                                                 1);  // use 1 spinner thread
  node.execute();
  return 0;
}
