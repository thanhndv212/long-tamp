#include "long_tamp_bt/event_logger.hpp"
#include "long_tamp_bt/python_task_session.hpp"
#include "long_tamp_bt/task_nodes.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include <iostream>
#include <memory>
#include <set>
#include <string>

int main(int argc, char** argv)
{
  std::string factory_name = "create_fake_session";
  std::string options = "{}";
  std::string events_path;
  for(int index = 1; index + 1 < argc; ++index)
  {
    if(std::string(argv[index]) == "--factory")
    {
      factory_name = argv[index + 1];
    }
    else if(std::string(argv[index]) == "--options")
    {
      options = argv[index + 1];
    }
    else if(std::string(argv[index]) == "--events")
    {
      events_path = argv[index + 1];
    }
  }

  try
  {
    const std::set<std::string> allowed_factories = {
      "create_fake_session", "create_twin_session", "create_twin_regrasp_session",
      "create_screw_session"
    };
    if(allowed_factories.count(factory_name) == 0)
    {
      throw std::runtime_error("session factory is not allowlisted: " + factory_name);
    }
    auto session = std::make_shared<long_tamp_bt::PythonTaskSession>(factory_name, options);
    BT::BehaviorTreeFactory factory;
    long_tamp_bt::RegisterNodes(factory);
    auto blackboard = BT::Blackboard::create();
    blackboard->set(long_tamp_bt::kSessionKey,
                    std::static_pointer_cast<long_tamp_bt::TaskSession>(session));
    auto tree = factory.createTreeFromText(session->behaviorTreeXml(), blackboard);
    BT::printTreeRecursively(tree.rootNode());
    BT::TreeObserver observer(tree);
    std::unique_ptr<long_tamp_bt::JsonlEventLogger> events;
    if(!events_path.empty())
    {
      events = std::make_unique<long_tamp_bt::JsonlEventLogger>(tree.rootNode(), events_path);
    }
    const auto status = tree.tickWhileRunning();
    std::cout << "Task plan status: " << BT::toStr(status) << '\n';
    std::cout << "Session report: " << session->report() << '\n';
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
  }
  catch(const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}