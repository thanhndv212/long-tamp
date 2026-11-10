#pragma once

#include "long_tamp_bt/task_session.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <string>
#include <vector>

namespace long_tamp_bt
{
// The node IDs the TaskPlan compiler emits (long_tamp.tasks.task_planning.compiler).
const std::vector<std::string>& NodeIds();

// Registers the task-planning nodes. Each node reads its session from the
// blackboard (kSessionKey), so the same registration serves any session.
// The plugin library (long_tamp_bt_nodes_plugin) does this in BT_REGISTER_NODES,
// for factory.registerFromPlugin(); link long_tamp_bt_nodes and call it to
// register them statically instead.
void RegisterNodes(BT::BehaviorTreeFactory& factory);
}  // namespace long_tamp_bt
