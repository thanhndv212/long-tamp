#pragma once

#include "python_session.hpp"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include <memory>

void RegisterTaskPlanningNodes(BT::BehaviorTreeFactory& factory,
                               std::shared_ptr<PythonSession> session);