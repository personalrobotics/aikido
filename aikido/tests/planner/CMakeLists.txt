add_subdirectory("parabolic")

if(${OMPL_FOUND})
  add_subdirectory("ompl")
endif()

aikido_add_test(test_SnapPlanner test_SnapPlanner.cpp)
target_link_libraries(test_SnapPlanner
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner")


aikido_add_test(test_VectorFieldPlanner test_VectorFieldPlanner.cpp)
target_link_libraries(test_VectorFieldPlanner
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner")

