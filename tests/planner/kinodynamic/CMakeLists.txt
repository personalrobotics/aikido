aikido_add_test(test_KinodynamicTimer
  test_KinodynamicTimer.cpp)
target_link_libraries(test_KinodynamicTimer
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner_parabolic"
  "${PROJECT_NAME}_planner_kinodynamic"
  "${PROJECT_NAME}_statespace")


