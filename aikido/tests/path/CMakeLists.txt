aikido_add_test(test_PiecewiseLinearTrajectory
  test_PiecewiseLinearTrajectory.cpp)
target_link_libraries(test_PiecewiseLinearTrajectory
  "${PROJECT_NAME}_path"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_SplineTrajectory2
  test_SplineTrajectory2.cpp)
target_link_libraries(test_SplineTrajectory2
  "${PROJECT_NAME}_path"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_Spline
  test_Spline.cpp)
target_link_libraries(test_Spline
  "${PROJECT_NAME}_path")

aikido_add_test(test_SplineProblem
  test_SplineProblem.cpp)
target_link_libraries(test_SplineProblem
  "${PROJECT_NAME}_path")
