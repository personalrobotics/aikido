#==============================================================================
# Dependencies
#
find_package(trajectory_msgs QUIET)
aikido_check_package(trajectory_msgs "aikido::control::ros" "trajectory_msgs")

#==============================================================================
# Libraries
#
set(sources
  Conversions.cpp
)

add_library("${PROJECT_NAME}_control_ros" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_control_ros" SYSTEM
  PUBLIC
    ${trajectory_msgs_INCLUDE_DIRS}
)

target_link_libraries("${PROJECT_NAME}_control_ros"
  "${PROJECT_NAME}_control"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_trajectory"
  ${DART_LIBRARIES}
  ${trajectory_msgs_LIBRARIES}
)

add_component(${PROJECT_NAME} control_ros)
add_component_targets(${PROJECT_NAME} control_ros "${PROJECT_NAME}_control_ros")
add_component_dependencies(${PROJECT_NAME} control_ros control statespace trajectory)

coveralls_add_sources(${sources})
