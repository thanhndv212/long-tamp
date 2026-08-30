#include "task_nodes.hpp"

#include <behaviortree_cpp/contrib/json.hpp>

namespace
{
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

class SessionCondition : public BT::ConditionNode
{
public:
  SessionCondition(const std::string& name, const BT::NodeConfig& config,
                   std::shared_ptr<PythonSession> session, std::string method,
                   std::string value_key)
    : BT::ConditionNode(name, config)
    , session_(std::move(session))
    , method_(std::move(method))
    , value_key_(std::move(value_key))
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("step_id") };
  }

  BT::NodeStatus tick() override
  {
    const auto step_id = getInput<std::string>("step_id");
    if(!step_id)
    {
      throw BT::RuntimeError(step_id.error());
    }
    const auto json = nlohmann::json::parse(session_->call(method_, step_id.value()));
    return json.value(value_key_, false) ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<PythonSession> session_;
  std::string method_;
  std::string value_key_;
};

class ExecuteTaskStep : public BT::SyncActionNode
{
public:
  ExecuteTaskStep(const std::string& name, const BT::NodeConfig& config,
                  std::shared_ptr<PythonSession> session)
    : BT::SyncActionNode(name, config), session_(std::move(session))
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("step_id") };
  }

  BT::NodeStatus tick() override
  {
    const auto step_id = getInput<std::string>("step_id");
    if(!step_id)
    {
      throw BT::RuntimeError(step_id.error());
    }
    return resultStatus(session_->call("execute_step", step_id.value()));
  }

private:
  std::shared_ptr<PythonSession> session_;
};
}  // namespace

void RegisterTaskPlanningNodes(BT::BehaviorTreeFactory& factory,
                               std::shared_ptr<PythonSession> session)
{
  factory.registerSimpleAction("SetupTaskPlan", [session](BT::TreeNode&) {
    return resultStatus(session->call("setup", "{}"));
  });
  factory.registerBuilder<SessionCondition>(
      "TaskStepComplete", BT::CreateBuilder<SessionCondition>(session,
                                                               "is_step_complete",
                                                               "complete"));
  factory.registerBuilder<SessionCondition>(
      "TaskStepReady", BT::CreateBuilder<SessionCondition>(session,
                                                            "check_precondition", "ready"));
    factory.registerBuilder<SessionCondition>(
      "TaskCapabilityCondition",
      BT::CreateBuilder<SessionCondition>(session, "evaluate_condition", "value"));
  factory.registerBuilder<ExecuteTaskStep>("ExecuteTaskStep",
                                            BT::CreateBuilder<ExecuteTaskStep>(session));
  factory.registerSimpleAction("FinalizeTaskPlan", [session](BT::TreeNode&) {
    return resultStatus(session->call("finalize"));
  });
}