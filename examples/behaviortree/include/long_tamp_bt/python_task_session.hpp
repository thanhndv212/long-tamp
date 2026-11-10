#pragma once

#include "long_tamp_bt/task_session.hpp"

#include <Python.h>

#include <mutex>
#include <string>

namespace long_tamp_bt
{
// A TaskSession run by embedded CPython: a long_tamp.tasks.task_planning.host
// factory (e.g. create_fake_session) builds the Python TaskPlanningSession,
// and each call goes to the method of the same name. Calls are serialized and
// take the interpreter lock.
class PythonTaskSession : public TaskSession
{
public:
  explicit PythonTaskSession(const std::string& factory, const std::string& options = "{}");
  ~PythonTaskSession() override;

  PythonTaskSession(const PythonTaskSession&) = delete;
  PythonTaskSession& operator=(const PythonTaskSession&) = delete;

  std::string behaviorTreeXml() override { return call("get_behavior_tree_xml"); }
  std::string setup(const std::string& options_json) override
  {
    return call("setup", options_json);
  }
  std::string isStepComplete(const std::string& step_id) override
  {
    return call("is_step_complete", step_id);
  }
  std::string checkPrecondition(const std::string& step_id) override
  {
    return call("check_precondition", step_id);
  }
  std::string evaluateCondition(const std::string& step_id) override
  {
    return call("evaluate_condition", step_id);
  }
  std::string executeStep(const std::string& step_id) override
  {
    return call("execute_step", step_id);
  }
  std::string finalize() override { return call("finalize"); }
  std::string report() override { return call("get_report"); }

  // Any method of the Python session, no argument or one string argument.
  std::string call(const std::string& method);
  std::string call(const std::string& method, const std::string& argument);

private:
  std::string callImpl(const std::string& method, const std::string* argument);
  static std::string pythonError();

  PyObject* session_ = nullptr;
  std::mutex mutex_;
};
}  // namespace long_tamp_bt
