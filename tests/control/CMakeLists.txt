aikido_add_test(test_InstantaneousTrajectoryExecutor
  test_InstantaneousTrajectoryExecutor.cpp)
target_link_libraries(test_InstantaneousTrajectoryExecutor
  "${PROJECT_NAME}_control")

aikido_add_test(test_KinematicSimulationTrajectoryExecutor
  test_KinematicSimulationTrajectoryExecutor.cpp)
target_link_libraries(test_KinematicSimulationTrajectoryExecutor
  "${PROJECT_NAME}_control")

aikido_add_test(test_KinematicSimulationPositionExecutor
  test_KinematicSimulationPositionExecutor.cpp)
target_link_libraries(test_KinematicSimulationPositionExecutor
  "${PROJECT_NAME}_control")

aikido_add_test(test_QueuedTrajectoryExecutor
  test_QueuedTrajectoryExecutor.cpp)
target_link_libraries(test_QueuedTrajectoryExecutor
  "${PROJECT_NAME}_control")

add_subdirectory("ros")

