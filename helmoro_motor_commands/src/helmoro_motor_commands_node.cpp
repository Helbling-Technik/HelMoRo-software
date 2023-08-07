#include "helmoro_motor_commands/helmoro_motor_commands.hpp"

// nodewrap
#include "any_node/Nodewrap.hpp"

int main(int argc, char **argv)
{
  any_node::Nodewrap<helmoro_motor_commands::HelmoroMotorCommands> node(argc, argv, "helmoro_motor_commands", 1);// use 1 spinner thread
  node.execute();
  return 0;
}
