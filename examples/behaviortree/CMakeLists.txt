# Configured on its own (cmake -S examples/behaviortree), this builds the nodes, the
# plugin, the host and their tests without the top-level project's HPP C++
# dependencies: none of these targets link HPP or pinocchio. The embedded Python
# finds long_tamp through PYTHONPATH (set on the tests below).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.22)
  project(long_tamp_behaviortree LANGUAGES CXX)
  include(CTest)
  get_filename_component(LONG_TAMP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
else()
  set(LONG_TAMP_SOURCE_DIR "${PROJECT_SOURCE_DIR}")
endif()

include(FetchContent)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

set(BTCPP_EXAMPLES OFF)
set(BTCPP_BUILD_TOOLS OFF)
set(BTCPP_GROOT_INTERFACE OFF)
set(BTCPP_SQLITE_LOGGING OFF)
set(_agimus_build_testing "${BUILD_TESTING}")
set(BUILD_TESTING OFF)

if(DEFINED BEHAVIORTREE_CPP_SOURCE_DIR)
  add_subdirectory(
    "${BEHAVIORTREE_CPP_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}/behaviortree_cpp"
    EXCLUDE_FROM_ALL)
else()
  FetchContent_Declare(
    behaviortree_cpp_source
    GIT_REPOSITORY https://github.com/BehaviorTree/BehaviorTree.CPP.git
    GIT_TAG c88a9f429a421b312599a07fa8902524b09bf90a
    GIT_SHALLOW FALSE)
  FetchContent_MakeAvailable(behaviortree_cpp_source)
endif()

set(BUILD_TESTING "${_agimus_build_testing}")

# The task-planning BT nodes (#25): a library to link (RegisterNodes), the
# same nodes as a BehaviorTree.CPP plugin (factory.registerFromPlugin, Nav2's
# plugin_lib_names), and the embedded-Python session. The nodes read their
# session (long_tamp_bt::TaskSession) from the root blackboard, so a session can
# be Python, ROS 2 services, or C++.
add_library(long_tamp_bt_nodes STATIC src/task_nodes.cpp src/event_logger.cpp)
target_include_directories(
  long_tamp_bt_nodes PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
target_link_libraries(long_tamp_bt_nodes PUBLIC BT::behaviortree_cpp)
target_compile_features(long_tamp_bt_nodes PUBLIC cxx_std_17)
set_target_properties(long_tamp_bt_nodes PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(long_tamp_bt_nodes_plugin SHARED src/plugin.cpp)
target_link_libraries(long_tamp_bt_nodes_plugin PRIVATE long_tamp_bt_nodes)
target_compile_definitions(long_tamp_bt_nodes_plugin PRIVATE BT_PLUGIN_EXPORT)

add_library(long_tamp_bt_python STATIC src/python_task_session.cpp)
target_include_directories(
  long_tamp_bt_python PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                             $<INSTALL_INTERFACE:include>)
target_link_libraries(long_tamp_bt_python PUBLIC Python3::Python)
target_compile_features(long_tamp_bt_python PUBLIC cxx_std_17)

add_executable(agimus_taskplan_bt src/main.cpp)
target_link_libraries(agimus_taskplan_bt PRIVATE long_tamp_bt_nodes long_tamp_bt_python)

# Example (#25): the screw-assembly mission in a stock factory that loads the
# nodes from the plugin (it links the event logger, not the nodes).
add_executable(screw_assembly_plugin src/screw_assembly_plugin.cpp)
target_link_libraries(screw_assembly_plugin PRIVATE long_tamp_bt_nodes long_tamp_bt_python)
target_compile_definitions(
  screw_assembly_plugin
  PRIVATE LONG_TAMP_BT_PLUGIN="$<TARGET_FILE:long_tamp_bt_nodes_plugin>")
add_dependencies(screw_assembly_plugin long_tamp_bt_nodes_plugin)

if(BUILD_TESTING)
  # #25: a stock factory loads the plugin and runs a tree against a C++ session;
  # the test links only BehaviorTree.CPP (the session interface is header-only).
  add_executable(long_tamp_bt_plugin_test test/plugin_loads.cpp)
  target_include_directories(long_tamp_bt_plugin_test PRIVATE include)
  target_link_libraries(long_tamp_bt_plugin_test PRIVATE BT::behaviortree_cpp)
  target_compile_features(long_tamp_bt_plugin_test PRIVATE cxx_std_17)
  add_test(NAME taskplan_bt_plugin_loads
           COMMAND long_tamp_bt_plugin_test $<TARGET_FILE:long_tamp_bt_nodes_plugin>)

  add_test(NAME taskplan_bt_fake COMMAND agimus_taskplan_bt --factory create_fake_session)

  # Failure-path coverage: each case exercises a distinct error boundary in
  # the CPython bridge / BT node layer (see host.py's create_fake_session
  # "fault" option docstring) and asserts the documented supervisor exit
  # code (see run_taskplan_bt_supervised.py's module docstring).
  add_test(
    NAME taskplan_bt_fault_capability_raises
    COMMAND agimus_taskplan_bt --factory create_fake_session
            --options {"fault":"capability_raises"})
  set_tests_properties(taskplan_bt_fault_capability_raises PROPERTIES
                       PASS_REGULAR_EXPRESSION "Task plan status: FAILURE")

  add_test(
    NAME taskplan_bt_fault_malformed_json
    COMMAND agimus_taskplan_bt --factory create_fake_session
            --options {"fault":"malformed_json_response"})
  set_tests_properties(taskplan_bt_fault_malformed_json PROPERTIES WILL_FAIL TRUE)

  add_test(
    NAME taskplan_bt_fault_missing_method
    COMMAND agimus_taskplan_bt --factory create_fake_session
            --options {"fault":"missing_method"})
  set_tests_properties(taskplan_bt_fault_missing_method PROPERTIES WILL_FAIL TRUE)

  # The C++ host's event stream matches the Python runner's, transition for
  # transition (same schema, same IR ids, roles, names and statuses).
  add_test(
    NAME taskplan_bt_events
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/check_events.py
            $<TARGET_FILE:agimus_taskplan_bt>)

  add_test(NAME taskplan_bt_unknown_factory
           COMMAND agimus_taskplan_bt --factory not_allowlisted)
  set_tests_properties(taskplan_bt_unknown_factory PROPERTIES WILL_FAIL TRUE)

  set_tests_properties(
    taskplan_bt_fake
    taskplan_bt_fault_capability_raises
    taskplan_bt_fault_malformed_json
    taskplan_bt_fault_missing_method
    taskplan_bt_unknown_factory
    taskplan_bt_events
    PROPERTIES ENVIRONMENT "PYTHONPATH=${LONG_TAMP_SOURCE_DIR}/src:$ENV{PYTHONPATH}")

  # Opt-in only: unlike the fake-session cases above, this drives a real
  # PyHPP scene (TWIN's bimanual lift-ball, script/twin/twin_bt_session.py)
  # through real RRT/IK planning -- ~2 minutes, and fails outright in any
  # environment without the PyHPP backend built (see docs/INSTALL.md).
  # Default OFF so a plain `-DBUILD_TESTING=ON` configure (e.g. CI without
  # PyHPP) never has to build or run it; enable explicitly in an
  # environment that has the real backend, same one create_twin_session
  # itself needs.
  option(BUILD_BEHAVIORTREE_REAL_MISSION_TESTS
         "Also run BT.CPP CTest cases against real PyHPP missions (needs the PyHPP backend)"
         OFF)
  if(BUILD_BEHAVIORTREE_REAL_MISSION_TESTS)
    add_test(NAME taskplan_bt_twin_lift_ball
             COMMAND agimus_taskplan_bt --factory create_twin_session)
    set_tests_properties(
      taskplan_bt_twin_lift_ball PROPERTIES
      PASS_REGULAR_EXPRESSION "Task plan status: SUCCESS"
      TIMEOUT 300
      ENVIRONMENT "PYTHONPATH=${LONG_TAMP_SOURCE_DIR}/src:$ENV{PYTHONPATH}")

    # §11 item 1: exercises a real release() forced by a fallback/condition
    # guard before a second real grasp() -- create_twin_session's flat
    # two-grasp document never releases anything, so this is the first
    # opt-in real-mission CTest to prove the compiler's Fallback/retry
    # shapes compose correctly one level above a single transaction.
    add_test(NAME taskplan_bt_twin_regrasp
             COMMAND agimus_taskplan_bt --factory create_twin_regrasp_session)
    set_tests_properties(
      taskplan_bt_twin_regrasp PROPERTIES
      PASS_REGULAR_EXPRESSION "Task plan status: SUCCESS"
      # Higher than taskplan_bt_twin_lift_ball's 300: this scenario RRT-plans
      # the same real edge TWICE (first grasp, then a forced release+regrasp
      # of the same target -- see create_twin_regrasp_session's docstring),
      # each draw independently taking anywhere up to ~115s observed, plus
      # up to 8 retries' worth of budget on the regrasp specifically.
      TIMEOUT 900
      ENVIRONMENT "PYTHONPATH=${LONG_TAMP_SOURCE_DIR}/src:$ENV{PYTHONPATH}")

    # #58: the screw-assembly cell, seeded (script/screw_assembly/
    # screw_bt_session.py): pick the driver, home it while the left arm
    # grasps part 1 -- a parallel node, BT.CPP's Parallel -- then rack it.
    add_test(NAME taskplan_bt_screw_cell
             COMMAND agimus_taskplan_bt --factory create_screw_session)
    set_tests_properties(
      taskplan_bt_screw_cell PROPERTIES
      PASS_REGULAR_EXPRESSION "Task plan status: SUCCESS"
      TIMEOUT 600
      ENVIRONMENT "PYTHONPATH=${LONG_TAMP_SOURCE_DIR}/src:$ENV{PYTHONPATH}")
  endif()
endif()

install(TARGETS agimus_taskplan_bt screw_assembly_plugin RUNTIME DESTINATION bin)
install(TARGETS long_tamp_bt_nodes_plugin LIBRARY DESTINATION lib)
install(DIRECTORY include/long_tamp_bt DESTINATION include)