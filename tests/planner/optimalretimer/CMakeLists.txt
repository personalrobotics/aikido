aikido_add_test(test_OptimalRetimer
  test_OptimalRetimer.cpp)
target_link_libraries(test_OptimalRetimer
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_planner_optimalretimer"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_OptimalRetimerPostProcessor
  test_OptimalRetimerPostProcessor.cpp)
target_link_libraries(test_OptimalRetimerPostProcessor
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_planner_optimalretimer"
  "${PROJECT_NAME}_statespace")
