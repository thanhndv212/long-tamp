#pragma once

#include <memory>
#include <string>

namespace long_tamp_bt
{
// What the task-planning BT nodes ask of a mission (#25). It mirrors the
// Python TaskPlanningSession (long_tamp.tasks.task_planning.session): every
// call takes and returns JSON text, so a session can live anywhere — in
// embedded Python (PythonTaskSession), behind ROS 2 services, or in C++.
//
// Responses (JSON objects):
//   setup / execute_step / finalize  {"status": "success" | "skipped" | "failure" |
//                                     "retry", "message": ...}
//   is_step_complete                 {"complete": bool}
//   check_precondition               {"ready": bool}
//   evaluate_condition               {"value": bool}
class TaskSession
{
public:
  virtual ~TaskSession() = default;

  // The compiled plan (BehaviorTree.CPP XML), for hosts that build the tree
  // from the session; the nodes don't call it.
  virtual std::string behaviorTreeXml() = 0;

  virtual std::string setup(const std::string& options_json) = 0;
  virtual std::string isStepComplete(const std::string& step_id) = 0;
  virtual std::string checkPrecondition(const std::string& step_id) = 0;
  virtual std::string evaluateCondition(const std::string& step_id) = 0;
  virtual std::string executeStep(const std::string& step_id) = 0;
  virtual std::string finalize() = 0;

  // A summary for the host (free-form JSON); the nodes don't call it.
  virtual std::string report() { return "{}"; }
};

using TaskSessionPtr = std::shared_ptr<TaskSession>;

// The blackboard entry the nodes read their session from. Put it on the
// tree's root blackboard before ticking:
//   blackboard->set(long_tamp_bt::kSessionKey, session);
// (like Nav2's "node" entry).
inline constexpr const char* kSessionKey = "long_tamp_session";
}  // namespace long_tamp_bt
