// The task-planning nodes as a BehaviorTree.CPP plugin (#25):
//   factory.registerFromPlugin("liblong_tamp_bt_nodes_plugin.so");
// or, in Nav2, list long_tamp_bt_nodes_plugin in plugin_lib_names. The nodes
// then read their session from the root blackboard (long_tamp_bt::kSessionKey).
#include "long_tamp_bt/task_nodes.hpp"

#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  long_tamp_bt::RegisterNodes(factory);
}
