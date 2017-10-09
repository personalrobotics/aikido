add_subdirectory("parabolic")
add_subdirectory("ompl")
add_subdirectory("optimization")

aikido_add_test(test_SnapPlanner test_SnapPlanner.cpp)
target_link_libraries(test_SnapPlanner
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner")
