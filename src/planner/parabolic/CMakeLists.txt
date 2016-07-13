aikido_add_component(planner_parabolic
  DEPENDENCIES planner util statespace trajectory)

include_directories(
  "${PROJECT_SOURCE_DIR}/src/external/hauser_parabolic_smoother")

aikido_add_library("${PROJECT_NAME}_planner_parabolic"
  COMPONENT planner_parabolic
  ParabolicTimer.cpp)
target_link_libraries("${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_hauserparabolicsmoother"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_util"
  "${PROJECT_NAME}_statespace")
