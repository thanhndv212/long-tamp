// #25's acceptance: the task-planning nodes load into a stock BehaviorTree.CPP
// factory from the plugin library, and run against a session written in C++
// (no Python): what a ROS 2 / Nav2 tree does. This program links only
// BehaviorTree.CPP; the session interface is header-only.
#include "long_tamp_bt/task_session.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <iostream>
#include <map>
#include <set>
#include <string>

namespace
{
int failures = 0;

void check(bool ok, const std::string& what)
{
  std::cout << (ok ? "ok   " : "FAIL ") << what << '\n';
  if(!ok)
  {
    ++failures;
  }
}

// A mission in C++: step "a" is already done, step "b" fails its first
// attempt, the guard "at-b" holds once "b" is done.
class FakeSession : public long_tamp_bt::TaskSession
{
public:
  std::map<std::string, int> calls;
  std::set<std::string> done = { "a" };
  int b_attempts = 0;

  std::string behaviorTreeXml() override { return ""; }
  std::string setup(const std::string&) override { return count("setup", R"({"status": "success"})"); }
  std::string isStepComplete(const std::string& step) override
  {
    return count("complete:" + step, done.count(step) ? R"({"complete": true})" : R"({"complete": false})");
  }
  std::string checkPrecondition(const std::string& step) override
  {
    return count("ready:" + step, R"({"ready": true})");
  }
  std::string evaluateCondition(const std::string& step) override
  {
    return count("condition:" + step, done.count("b") ? R"({"value": true})" : R"({"value": false})");
  }
  std::string executeStep(const std::string& step) override
  {
    if(step == "b" && ++b_attempts == 1)
    {
      return count("execute:" + step, R"({"status": "retry", "message": "first attempt"})");
    }
    done.insert(step);
    return count("execute:" + step, R"({"status": "success"})");
  }
  std::string finalize() override { return count("finalize", R"({"status": "success"})"); }

private:
  std::string count(const std::string& key, std::string response)
  {
    ++calls[key];
    return response;
  }
};

// The shape the TaskPlan compiler emits: setup, transactions (complete? or
// ready -> retry -> execute), a capability condition, finalize.
const char* kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="Mission">
  <BehaviorTree ID="Mission">
    <Sequence>
      <SetupTaskPlan/>
      <Fallback name="a transaction">
        <TaskStepComplete step_id="a"/>
        <Sequence>
          <TaskStepReady step_id="a"/>
          <RetryUntilSuccessful num_attempts="2"><ExecuteTaskStep step_id="a"/></RetryUntilSuccessful>
        </Sequence>
      </Fallback>
      <SubTree ID="StepB"/>
      <TaskCapabilityCondition step_id="at-b"/>
      <FinalizeTaskPlan/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="StepB">
    <Fallback name="b transaction">
      <TaskStepComplete step_id="b"/>
      <Sequence>
        <TaskStepReady step_id="b"/>
        <RetryUntilSuccessful num_attempts="3"><ExecuteTaskStep step_id="b"/></RetryUntilSuccessful>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>)";
}  // namespace

int main(int argc, char** argv)
{
  if(argc < 2)
  {
    std::cerr << "usage: " << argv[0] << " <path to the plugin library>\n";
    return 2;
  }
  BT::BehaviorTreeFactory factory;  // stock: nothing of ours registered
  factory.registerFromPlugin(argv[1]);
  for(const char* id : { "SetupTaskPlan", "TaskStepComplete", "TaskStepReady",
                         "TaskCapabilityCondition", "ExecuteTaskStep", "FinalizeTaskPlan" })
  {
    check(factory.manifests().count(id) == 1, std::string("plugin registers ") + id);
  }

  auto session = std::make_shared<FakeSession>();
  auto blackboard = BT::Blackboard::create();
  blackboard->set(long_tamp_bt::kSessionKey,
                  std::static_pointer_cast<long_tamp_bt::TaskSession>(session));
  auto tree = factory.createTreeFromText(kTree, blackboard);
  const auto status = tree.tickWhileRunning();
  check(status == BT::NodeStatus::SUCCESS, "the tree succeeds");
  check(session->calls["execute:a"] == 0, "a step whose effect holds is skipped");
  check(session->calls["execute:b"] == 2, "a retried step runs again (2 attempts)");
  check(session->calls["setup"] == 1 && session->calls["finalize"] == 1, "setup and finalize run once");
  check(session->calls["condition:at-b"] == 1, "the capability condition is asked");

  // Without a session on the blackboard: a clear error, not a crash.
  auto bare = factory.createTreeFromText(kTree);
  try
  {
    bare.tickWhileRunning();
    check(false, "a tree without a session throws");
  }
  catch(const std::exception& error)
  {
    check(std::string(error.what()).find("no task session") != std::string::npos,
          std::string("a tree without a session throws: ") + error.what());
  }
  std::cout << (failures ? "FAILED" : "PASSED") << '\n';
  return failures ? 1 : 0;
}
