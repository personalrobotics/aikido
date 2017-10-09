if(NOT TARGET "${PROJECT_NAME}_planner_optimization")
  return()
endif()

aikido_add_test(test_OptimizationBasedMotionPlanner
  test_OptimizationBasedMotionPlanner.cpp)
target_link_libraries(test_OptimizationBasedMotionPlanner
  "${PROJECT_NAME}_planner_optimization")
