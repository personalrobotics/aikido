aikido_add_test(test_KunzRetimer
  test_KunzRetimer.cpp)
target_link_libraries(test_KunzRetimer
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_planner_kunzretimer"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_KunzRetimerPostProcessor
  test_KunzRetimerPostProcessor.cpp)
target_link_libraries(test_KunzRetimerPostProcessor
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_planner_kunzretimer"
  "${PROJECT_NAME}_statespace")
