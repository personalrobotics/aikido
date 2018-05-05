if(NOT TARGET "${PROJECT_NAME}_planner_kinodynamics")
  return()
endif()

aikido_add_test(test_KinodynamicPlanner test_KinodynamicPlanner.cpp)
target_link_libraries(test_KinodynamicPlanner
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_planner"
  "${PROJECT_NAME}_planner_kinodynamics")
