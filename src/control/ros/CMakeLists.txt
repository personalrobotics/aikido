#==============================================================================
# Dependencies
#

find_package(actionlib QUIET)
aikido_check_package(actionlib "aikido::control::ros" "actionlib")
aikido_include_directories(${actionlib_INCLUDE_DIRS})

find_package(control_msgs QUIET)
aikido_check_package(control_msgs "aikido::control::ros" "control_msgs")
aikido_include_directories(${control_msgs_INCLUDE_DIRS})

find_package(roscpp QUIET)
aikido_check_package(roscpp "aikido::control::ros" "roscpp")
aikido_include_directories(${roscpp_INCLUDE_DIRS})

find_package(trajectory_msgs QUIET)
aikido_check_package(trajectory_msgs "aikido::control::ros" "trajectory_msgs")
aikido_include_directories(${trajectory_msgs_INCLUDE_DIRS})

#==============================================================================
# Libraries
#
aikido_add_library("${PROJECT_NAME}_control_ros"
  Conversions.cpp
  RosJointStateClient.cpp
  RosTrajectoryExecutor.cpp
)
target_link_libraries("${PROJECT_NAME}_control_ros"
  "${PROJECT_NAME}_control"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_trajectory"
  ${DART_LIBRARIES}
  ${actionlib_LIBRARIES}
  ${control_msgs_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
)
