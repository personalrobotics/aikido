aikido_add_test(test_KinematicSimulationTrajectoryExecutor
  test_KinematicSimulationTrajectoryExecutor.cpp)

target_link_libraries(test_KinematicSimulationTrajectoryExecutor
  "${PROJECT_NAME}_control")

aikido_add_test(test_BarrettFingerSpreadCommandExecutor
  test_BarrettFingerSpreadCommandExecutor.cpp)

target_link_libraries(test_BarrettFingerSpreadCommandExecutor
  "${PROJECT_NAME}_control")


aikido_add_test(test_BarrettFingerPositionCommandExecutor
  test_BarrettFingerPositionCommandExecutor.cpp)

target_link_libraries(test_BarrettFingerPositionCommandExecutor
  "${PROJECT_NAME}_control")

aikido_add_test(test_BarrettHandPositionCommandExecutor
  test_BarrettHandPositionCommandExecutor.cpp)

target_link_libraries(test_BarrettHandPositionCommandExecutor
  "${PROJECT_NAME}_control")

add_subdirectory("ros")

