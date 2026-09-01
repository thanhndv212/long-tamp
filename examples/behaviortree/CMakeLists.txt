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

add_executable(
  agimus_taskplan_bt
  src/main.cpp
  src/python_session.cpp
  src/task_nodes.cpp)
target_include_directories(agimus_taskplan_bt PRIVATE include)
target_link_libraries(
  agimus_taskplan_bt
  PRIVATE
    BT::behaviortree_cpp
    Python3::Python)
target_compile_features(agimus_taskplan_bt PRIVATE cxx_std_17)

if(BUILD_TESTING)
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

  add_test(NAME taskplan_bt_unknown_factory
           COMMAND agimus_taskplan_bt --factory not_allowlisted)
  set_tests_properties(taskplan_bt_unknown_factory PROPERTIES WILL_FAIL TRUE)

  set_tests_properties(
    taskplan_bt_fake
    taskplan_bt_fault_capability_raises
    taskplan_bt_fault_malformed_json
    taskplan_bt_fault_missing_method
    taskplan_bt_unknown_factory
    PROPERTIES ENVIRONMENT "PYTHONPATH=${PROJECT_SOURCE_DIR}/src:$ENV{PYTHONPATH}")
endif()

install(TARGETS agimus_taskplan_bt RUNTIME DESTINATION bin)