set(sources ParabolicTimer.cpp)

add_library("${PROJECT_NAME}_planner_parabolic" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_planner_parabolic"
  PUBLIC
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_util"
    "${PROJECT_NAME}_statespace"
  PRIVATE
    "${PROJECT_NAME}_external_hauserparabolicsmoother"
)

add_component(${PROJECT_NAME} planner_parabolic)
add_component_targets(${PROJECT_NAME} planner_parabolic "${PROJECT_NAME}_planner_parabolic")
add_component_dependencies(${PROJECT_NAME} planner_parabolic
  planner
  statespace
  trajectory
  util
)

coveralls_add_sources(${sources})
