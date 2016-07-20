add_library("${PROJECT_NAME}_planner_parabolic" SHARED
  ParabolicTimer.cpp
)
set_property(TARGET "${PROJECT_NAME}_planner_parabolic" APPEND PROPERTY
  INCLUDE_DIRECTORIES "${hauserparabolicsmoother_INCLUDE_DIRS}"
)
target_link_libraries("${PROJECT_NAME}_planner_parabolic"
  PUBLIC
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_util"
    "${PROJECT_NAME}_statespace"
  PRIVATE
    ${hauserparabolicsmoother_LIBRARIES}
)

add_component(planner_parabolic)
add_component_targets(planner_parabolic "${PROJECT_NAME}_planner_parabolic")
add_component_dependencies(planner_parabolic
  planner
  statespace
  trajectory
  util
)
