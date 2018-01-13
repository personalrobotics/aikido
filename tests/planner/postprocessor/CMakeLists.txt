aikido_add_test(test_SmoothPostProcessor
  test_SmoothPostProcessor.cpp)
target_link_libraries(test_SmoothPostProcessor
  "${PROJECT_NAME}_planner_postprocessor"
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_statespace")
