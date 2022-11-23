#==============================================================================
# Dependencies
#
find_package(urdf QUIET)
aikido_check_package(urdf "aikido::robot::ros" "urdf")

find_package(srdfdom QUIET)
aikido_check_package(srdfdom "aikido::robot::ros" "srdfdom")

#==============================================================================
# Libraries
#
set(sources
  RosRobot.cpp
)

add_library("${PROJECT_NAME}_robot_ros" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_robot_ros" SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
    ${urdf_INCLUDE_DIRS}
    ${srdfdom_INCLUDE_DIRS}
)

target_link_libraries("${PROJECT_NAME}_robot_ros"
  PUBLIC
  "${PROJECT_NAME}_robot"
  "${PROJECT_NAME}_common"
  "${PROJECT_NAME}_io"
  "${PROJECT_NAME}_planner"
  "${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_planner_kunzretimer"
  "${PROJECT_NAME}_planner_vectorfield"
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace"
  ${DART_LIBRARIES}
  ${srdfdom_LIBRARIES}
  ${urdf_LIBRARIES}
)
target_compile_definitions("${PROJECT_NAME}_robot_ros"
  PUBLIC AIKIDO_HAS_ROBOT_ROS
)

add_component(${PROJECT_NAME} robot_ros)
add_component_targets(${PROJECT_NAME} robot_ros "${PROJECT_NAME}_robot_ros")
add_component_dependencies(${PROJECT_NAME} robot_ros 
  robot
  common
  io
  planner
  planner_ompl
  planner_parabolic
  planner_vectorfield
  constraint
  distance
  statespace
  trajectory)

clang_format_add_sources(${sources})
