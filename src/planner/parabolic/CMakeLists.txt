open_component(planner_parabolic
  DEPENDENCIES
    planner
    util
    statespace
    trajectory
)


add_component_library("${PROJECT_NAME}_planner_parabolic"
  COMPONENT planner_parabolic
  SOURCES ParabolicTimer.cpp
)
target_include_directories("${PROJECT_NAME}_planner_parabolic"
  PRIVATE ${hauserparabolicsmoother_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_parabolic"
  PUBLIC
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_util"
    "${PROJECT_NAME}_statespace"
  PRIVATE
    ${hauserparabolicsmoother_LIBRARIES}
)

close_component(planner_parabolic)
