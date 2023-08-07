#include "helmoro_navigation_goals/HelmoroNavigationGoals.hpp"

// nodewrap
#include "any_node/Nodewrap.hpp"

int main(int argc, char **argv)
{
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  any_node::Nodewrap<helmoro_navigation_goals::HelmoroNavigationGoals> node(argc, argv, "helmoro_navigation_goals",
                                                                            1);  // use 1 spinner thread
  node.execute();
  return 0;
}
