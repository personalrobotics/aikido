#==============================================================================
# Dependencies
#
find_package(roscpp QUIET)
aikido_check_package(roscpp "aikido::control::ros" "roscpp")

find_package(trajectory_msgs QUIET)
aikido_check_package(trajectory_msgs "aikido::control::ros" "trajectory_msgs")

# Note: Intentionally use "Module" mode because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.
find_package(actionlib QUIET MODULE)
aikido_check_package(actionlib "aikido::control::ros" "actionlib")

# Note: Intentionally use "Module" mode because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.
find_package(control_msgs QUIET MODULE)
aikido_check_package(control_msgs "aikido::control::ros" "control_msgs")

# Note: Intentionally use "Module" mode because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.
find_package(pr_control_msgs QUIET MODULE)
aikido_check_package(pr_control_msgs "aikido::control::ros" "pr_control_msgs")

#==============================================================================
# Libraries
#
set(sources
  RosTrajectoryExecutor.cpp
  RosTrajectoryExecutionException.cpp
  Conversions.cpp
  RosJointStateClient.cpp
  RosPositionCommandExecutor.cpp
  RosGripperCommandExecutor.cpp
)

add_library("${PROJECT_NAME}_control_ros" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_control_ros" SYSTEM
  PUBLIC
    ${actionlib_INCLUDE_DIRS}
    ${control_msgs_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${trajectory_msgs_INCLUDE_DIRS}
    ${actionlib_INCLUDE_DIRS}
    ${control_msgs_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${pr_control_msgs_INCLUDE_DIRS}
)

target_link_libraries("${PROJECT_NAME}_control_ros"
  PUBLIC
  "${PROJECT_NAME}_control"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_trajectory"
  ${DART_LIBRARIES}
  ${actionlib_LIBRARIES}
  ${control_msgs_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
  ${actionlib_LIBRARIES}
  ${control_msgs_LIBRARIES}
  ${roscpp_LIBRARIES}
)

add_component(${PROJECT_NAME} control_ros)
add_component_targets(${PROJECT_NAME} control_ros "${PROJECT_NAME}_control_ros")
add_component_dependencies(${PROJECT_NAME} control_ros control statespace trajectory)

clang_format_add_sources(${sources})
