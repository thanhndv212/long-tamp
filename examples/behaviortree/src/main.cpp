#include "python_session.hpp"
#include "task_nodes.hpp"

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
  }

  try
  {
    const std::set<std::string> allowed_factories = { "create_fake_session" };
    if(allowed_factories.count(factory_name) == 0)
    {
      throw std::runtime_error("session factory is not allowlisted: " + factory_name);
    }
    auto session = std::make_shared<PythonSession>(factory_name, options);
    BT::BehaviorTreeFactory factory;
    RegisterTaskPlanningNodes(factory, session);
    auto tree = factory.createTreeFromText(session->call("get_behavior_tree_xml"));
    BT::printTreeRecursively(tree.rootNode());
    BT::TreeObserver observer(tree);
    const auto status = tree.tickWhileRunning();
    std::cout << "Task plan status: " << BT::toStr(status) << '\n';
    std::cout << "Session report: " << session->call("get_report") << '\n';
    return status == BT::NodeStatus::SUCCESS ? 0 : 1;
  }
  catch(const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 2;
  }
}