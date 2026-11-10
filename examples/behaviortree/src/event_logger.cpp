#include "long_tamp_bt/event_logger.hpp"

#include <behaviortree_cpp/contrib/json.hpp>
#include <behaviortree_cpp/tree_node.h>

#include <chrono>
#include <stdexcept>

namespace long_tamp_bt
{
// The subscribing constructor exists in every 4.x release. Subscribing doesn't tick
// the tree, so no callback runs before the file below is open.
JsonlEventLogger::JsonlEventLogger(BT::TreeNode* root_node, const std::string& path)
  : BT::StatusChangeLogger(root_node), file_(path, std::ios::app)
{
  if(!file_)
  {
    throw std::runtime_error("cannot open event log: " + path);
  }
  setTimestampType(BT::TimestampType::absolute);
}

JsonlEventLogger::~JsonlEventLogger()
{
  unsubscribe(*this, 0);
  flush();
}

void JsonlEventLogger::callback(BT::Duration timestamp, const BT::TreeNode& node,
                                BT::NodeStatus prev_status, BT::NodeStatus status)
{
  const auto& attributes = node.config().other_attributes;
  const auto ir_id = attributes.find("_ir_id");
  const auto role = attributes.find("_ir_role");
  if(ir_id == attributes.end() || role == attributes.end() ||
     status == BT::NodeStatus::IDLE)
  {
    return;  // not a plan node (setup, finalize, the root sequence), or a reset
  }
  const nlohmann::json event = {
    { "schema", "long-tamp.events/1" },
    { "t", std::chrono::duration<double>(timestamp).count() },
    { "source", "bt" },
    { "ir_id", ir_id->second },
    { "role", role->second },
    { "name", node.name() },
    { "status", BT::toStr(status) },
    { "previous", BT::toStr(prev_status) },
  };
  std::lock_guard<std::mutex> lock(mutex_);
  // Flushed per event so a killed mission keeps every event up to the kill.
  file_ << event.dump() << '\n' << std::flush;
}

void JsonlEventLogger::flush()
{
  std::lock_guard<std::mutex> lock(mutex_);
  file_.flush();
}
}  // namespace long_tamp_bt
