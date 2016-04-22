aikido_add_test(test_Interpolated test_Interpolated.cpp)
target_link_libraries(test_Interpolated
  "${PROJECT_NAME}_path"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_SplineTrajectory2
  test_SplineTrajectory2.cpp)
target_link_libraries(test_SplineTrajectory2
  "${PROJECT_NAME}_path"
  "${PROJECT_NAME}_statespace")
