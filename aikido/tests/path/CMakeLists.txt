aikido_add_test(
    test_PiecewiseLinearTrajectory test_PiecewiseLinearTrajectory.cpp)
    target_link_libraries(test_PiecewiseLinearTrajectory
                          "${PROJECT_NAME}_distance"
                          "${PROJECT_NAME}_path"
                          "${PROJECT_NAME}_statespace")