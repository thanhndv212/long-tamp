// Example (#25): the screw-assembly mission run by a stock BehaviorTree.CPP
// factory that loads long-tamp's nodes from the plugin library, as your own
// application (or a ROS 2 / Nav2 tree) would:
//
//   1. load the nodes:      factory.registerFromPlugin(<liblong_tamp_bt_nodes_plugin.so>)
//   2. make a session:      here the Python screw-assembly session (planning on
//                           the real HPP scene); yours may be C++ or ROS 2
//   3. put it on the root blackboard (long_tamp_bt::kSessionKey)
//   4. build the compiled plan's tree and tick it, logging the event stream
//
//   screw_assembly_plugin [--plan short|full] [--seed N] [--events FILE]
//                         [--plugin PATH]
//
// The event stream (long-tamp.events/1) starts with a ``plan`` event, so
// `python -m long_tamp.viewer replay FILE` (or `serve`) draws the plan tree.
#include "long_tamp_bt/event_logger.hpp"
#include "long_tamp_bt/python_task_session.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/contrib/json.hpp>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#ifndef LONG_TAMP_BT_PLUGIN
#define LONG_TAMP_BT_PLUGIN "liblong_tamp_bt_nodes_plugin.so"
#endif

int main(int argc, char** argv)
{
  std::string plan = "short", seed = "1", events_path, plugin = LONG_TAMP_BT_PLUGIN;
  for(int i = 1; i + 1 < argc; i += 2)
  {
    const std::string flag = argv[i], value = argv[i + 1];
    if(flag == "--plan") plan = value;
    else if(flag == "--seed") seed = value;
    else if(flag == "--events") events_path = value;
    else if(flag == "--plugin") plugin = value;
    else
    {
      std::cerr << "unknown option " << flag << '\n';
      return 2;
    }
  }
  try
  {
    // 1. A stock factory: our nodes come from the plugin.
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin(plugin);
    std::cout << "loaded the task-planning nodes from " << plugin << '\n';

    // 2. The session: the screw-assembly cell, planned on the real scene.
    const nlohmann::json options = { { "plan", plan }, { "seed", std::stoi(seed) } };
    auto session = std::make_shared<long_tamp_bt::PythonTaskSession>("create_screw_session",
                                                                      options.dump());

    // 3. On the root blackboard, where every node (and subtree) finds it.
    auto blackboard = BT::Blackboard::create();
    blackboard->set(long_tamp_bt::kSessionKey,
                    std::static_pointer_cast<long_tamp_bt::TaskSession>(session));

    // 4. The compiled plan as a tree; the event stream starts with the plan.
    auto tree = factory.createTreeFromText(session->behaviorTreeXml(), blackboard);
    std::unique_ptr<long_tamp_bt::JsonlEventLogger> events;
    if(!events_path.empty())
    {
      const auto document = nlohmann::json::parse(session->call("get_plan_document"));
      const auto now = std::chrono::duration<double>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
      const nlohmann::json plan_event = {
        { "schema", "long-tamp.events/1" }, { "t", now },
        { "source", "bt" },                 { "ir_id", document["root"]["id"] },
        { "role", "plan" },                 { "name", document.value("mission_id", "mission") },
        { "status", "SUCCESS" },            { "previous", "IDLE" },
        { "plan", document },
      };
      std::ofstream(events_path, std::ios::trunc) << plan_event.dump() << '\n';
      events = std::make_unique<long_tamp_bt::JsonlEventLogger>(tree.rootNode(), events_path);
    }
    std::cout << "running the " << plan << " screw-assembly plan (seed " << seed << ")\n";
    const auto t0 = std::chrono::steady_clock::now();
    const auto status = tree.tickWhileRunning();
    const auto seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "Task plan status: " << BT::toStr(status) << " in " << seconds << " s\n";
    std::cout << "Session report: " << session->report() << '\n';
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
  }
  catch(const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
