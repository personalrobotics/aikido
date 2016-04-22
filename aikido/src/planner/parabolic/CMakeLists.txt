include_directories(
  "${PROJECT_SOURCE_DIR}/src/external/hauser_parabolic_smoother")

aikido_add_library("${PROJECT_NAME}_planner_parabolic"
  ParabolicTimer.cpp)
target_link_libraries("${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_hauserparabolicsmoother"
  "${PROJECT_NAME}_path"
  "${PROJECT_NAME}_util"
  "${PROJECT_NAME}_statespace")
