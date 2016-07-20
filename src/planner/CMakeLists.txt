add_library("${PROJECT_NAME}_planner" SHARED
  SnapPlanner.cpp
)
target_link_libraries("${PROJECT_NAME}_planner"
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_util"
)

add_component(planner)
add_component_targets(planner "${PROJECT_NAME}_planner")
add_component_dependencies(planner
  constraint
  distance
  statespace
  trajectory
  util
)

add_subdirectory("ompl")
add_subdirectory("parabolic")
