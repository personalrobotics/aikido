if(NOT TARGET "${PROJECT_NAME}_planner_vectorfield")
  return()
endif()

aikido_add_test(test_VectorFieldPlanner test_VectorFieldPlanner.cpp)
target_link_libraries(test_VectorFieldPlanner
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner"
  "${PROJECT_NAME}_planner_dart"
  "${PROJECT_NAME}_planner_vectorfield")
