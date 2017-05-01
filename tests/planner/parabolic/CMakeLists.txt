aikido_add_test(test_ParabolicTimer
  test_ParabolicTimer.cpp)
target_link_libraries(test_ParabolicTimer
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_ParabolicSmoother
  test_ParabolicSmoother.cpp)
target_link_libraries(test_ParabolicSmoother
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_statespace")
