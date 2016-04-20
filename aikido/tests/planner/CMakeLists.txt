add_subdirectory("parabolic")

aikido_add_test(test_SnapPlanner test_SnapPlanner.cpp)
target_link_libraries(test_SnapPlanner
                      "${PROJECT_NAME}_constraint"
                      "${PROJECT_NAME}_distance"
                      "${PROJECT_NAME}_path")
#TODO : This should be _planner, once we have a planner library.
