aikido_add_test(test_KinematicSimulationTrajectoryExecutor
  test_KinematicSimulationTrajectoryExecutor.cpp)

target_link_libraries(test_KinematicSimulationTrajectoryExecutor
  "${PROJECT_NAME}_control")

aikido_add_test(test_FingerSimulationStepExecutor
  test_FingerSimulationStepExecutor.cpp)

target_link_libraries(test_FingerSimulationStepExecutor
  "${PROJECT_NAME}_control")

aikido_add_test(test_HandSimulationTrajectoryExecutor
  test_HandSimulationTrajectoryExecutor.cpp)

target_link_libraries(test_HandSimulationTrajectoryExecutor
  "${PROJECT_NAME}_control")
