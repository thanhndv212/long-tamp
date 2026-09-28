#pragma once

#include <behaviortree_cpp/loggers/abstract_logger.h>

#include <fstream>
#include <mutex>
#include <string>

// Writes the mission event stream (long_tamp.tasks.task_planning.events, schema
// "long-tamp.events/1"), one JSON object per line, for every status change of a
// node the compiler stamped with _ir_id / _ir_role. The Python executor writes
// the same schema, so a mission reads the same whichever runs it.
class JsonlEventLogger : public BT::StatusChangeLogger
{
public:
  JsonlEventLogger(BT::TreeNode* root_node, const std::string& path);
  ~JsonlEventLogger() override;

  void callback(BT::Duration timestamp, const BT::TreeNode& node,
                BT::NodeStatus prev_status, BT::NodeStatus status) override;
  void flush() override;

private:
  std::ofstream file_;
  std::mutex mutex_;
};
