aikido_add_test(test_GeometricStateSpace test_GeometricStateSpace.cpp)
target_link_libraries(test_GeometricStateSpace
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_GoalRegion test_GoalRegion.cpp)
target_link_libraries(test_GoalRegion
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_MotionValidator test_MotionValidator.cpp)
target_link_libraries(test_MotionValidator
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_StateSampler test_StateSampler.cpp)
target_link_libraries(test_StateSampler
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_ValidityChecker test_ValidityChecker.cpp)
target_link_libraries(test_ValidityChecker
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_OMPLPlanner test_OMPLPlanner.cpp)
target_link_libraries(test_OMPLPlanner
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace")
