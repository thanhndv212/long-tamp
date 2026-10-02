#include "long_tamp_bt/task_nodes.hpp"

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/contrib/json.hpp>

namespace long_tamp_bt
{
namespace
{
// The session on the tree's root blackboard (so subtrees see it too).
TaskSessionPtr sessionOf(const BT::TreeNode& node)
{
  TaskSessionPtr session;
  const auto& blackboard = node.config().blackboard;
  if(!blackboard || !blackboard->rootBlackboard()->get(kSessionKey, session) || !session)
  {
    throw BT::RuntimeError(node.name(), ": no task session on the blackboard (set \"",
                           kSessionKey, "\" on the root blackboard before ticking)");
  }
  return session;
}

BT::NodeStatus resultStatus(const std::string& response)
{
  const auto json = nlohmann::json::parse(response);
  const auto status = json.value("status", "failure");
  if(status == "success")
  {
    return BT::NodeStatus::SUCCESS;
  }
  if(status == "skipped")
  {
    return BT::NodeStatus::SKIPPED;
  }
  return BT::NodeStatus::FAILURE;
}

std::string stepId(const BT::TreeNode& node, const BT::Expected<std::string>& step_id)
{
  if(!step_id)
  {
    throw BT::RuntimeError(node.name(), ": ", step_id.error());
  }
  return step_id.value();
}

// A condition answered by the session: SUCCESS when ``key`` is true.
template <std::string (TaskSession::*Method)(const std::string&)>
class SessionCondition : public BT::ConditionNode
{
public:
  SessionCondition(const std::string& name, const BT::NodeConfig& config, std::string key)
    : BT::ConditionNode(name, config), key_(std::move(key))
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("step_id") };
  }

  BT::NodeStatus tick() override
  {
    const auto session = sessionOf(*this);
    const auto id = stepId(*this, getInput<std::string>("step_id"));
    const auto json = nlohmann::json::parse(((*session).*Method)(id));
    return json.value(key_, false) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  std::string key_;
};

using TaskStepComplete = SessionCondition<&TaskSession::isStepComplete>;
using TaskStepReady = SessionCondition<&TaskSession::checkPrecondition>;
using TaskCapabilityCondition = SessionCondition<&TaskSession::evaluateCondition>;

class ExecuteTaskStep : public BT::SyncActionNode
{
public:
  ExecuteTaskStep(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("step_id") };
  }

  BT::NodeStatus tick() override
  {
    const auto session = sessionOf(*this);
    return resultStatus(session->executeStep(stepId(*this, getInput<std::string>("step_id"))));
  }
};

class SetupTaskPlan : public BT::SyncActionNode
{
public:
  SetupTaskPlan(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("options", "{}", "setup options (JSON)") };
  }

  BT::NodeStatus tick() override
  {
    const auto options = getInput<std::string>("options");
    return resultStatus(sessionOf(*this)->setup(options ? options.value() : "{}"));
  }
};

class FinalizeTaskPlan : public BT::SyncActionNode
{
public:
  FinalizeTaskPlan(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    return resultStatus(sessionOf(*this)->finalize());
  }
};
}  // namespace

const std::vector<std::string>& NodeIds()
{
  static const std::vector<std::string> ids = {
    "SetupTaskPlan",   "TaskStepComplete", "TaskStepReady",
    "TaskCapabilityCondition", "ExecuteTaskStep",  "FinalizeTaskPlan"
  };
  return ids;
}

void RegisterNodes(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<SetupTaskPlan>("SetupTaskPlan");
  factory.registerNodeType<TaskStepComplete>("TaskStepComplete", std::string("complete"));
  factory.registerNodeType<TaskStepReady>("TaskStepReady", std::string("ready"));
  factory.registerNodeType<TaskCapabilityCondition>("TaskCapabilityCondition",
                                                    std::string("value"));
  factory.registerNodeType<ExecuteTaskStep>("ExecuteTaskStep");
  factory.registerNodeType<FinalizeTaskPlan>("FinalizeTaskPlan");
}
}  // namespace long_tamp_bt
