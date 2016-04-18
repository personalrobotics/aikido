aikido_add_test(test_OMPLPlanner test_OMPLPlanner.cpp)
    target_link_libraries(test_OMPLPlanner
                          "${PROJECT_NAME}_distance"
                          "${PROJECT_NAME}_ompl"
                          "${PROJECT_NAME}_path"
                          "${PROJECT_NAME}_statespace")

aikido_add_test(test_OMPLGoalSets test_OMPLGoalSets.cpp)
    target_link_libraries(test_OMPLGoalSets
                          "${PROJECT_NAME}_distance"
                          "${PROJECT_NAME}_ompl"
                          "${PROJECT_NAME}_path"
                          "${PROJECT_NAME}_statespace")

aikido_add_test(test_GetMeasure test_GetMeasure.cpp)
    target_link_libraries(test_GetMeasure
                          "${PROJECT_NAME}_distance"
                          "${PROJECT_NAME}_ompl"
                          "${PROJECT_NAME}_path"
                          "${PROJECT_NAME}_statespace")
