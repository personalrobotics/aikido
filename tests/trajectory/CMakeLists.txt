aikido_add_test(test_Interpolated test_Interpolated.cpp)
target_link_libraries(test_Interpolated
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_SplineTrajectory test_SplineTrajectory.cpp)
target_link_libraries(test_SplineTrajectory
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_BSpline test_BSpline.cpp)
target_link_libraries(test_BSpline
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")
