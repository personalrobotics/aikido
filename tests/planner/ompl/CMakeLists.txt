if(NOT TARGET "${PROJECT_NAME}_planner_ompl")
  return()
endif()

aikido_add_test(test_GeometricStateSpace test_GeometricStateSpace.cpp)
target_link_libraries(test_GeometricStateSpace "${PROJECT_NAME}_planner_ompl")

#aikido_add_test(test_GoalRegion test_GoalRegion.cpp)
#target_link_libraries(test_GoalRegion "${PROJECT_NAME}_planner_ompl")

#aikido_add_test(test_MotionValidator test_MotionValidator.cpp)
#target_link_libraries(test_MotionValidator "${PROJECT_NAME}_planner_ompl")

#aikido_add_test(test_OMPLPlanner test_OMPLPlanner.cpp)
#target_link_libraries(test_OMPLPlanner "${PROJECT_NAME}_planner_ompl")

aikido_add_test(test_OMPLConfigurationToConfigurationPlanner test_OMPLConfigurationToConfigurationPlanner.cpp)
target_link_libraries(test_OMPLConfigurationToConfigurationPlanner "${PROJECT_NAME}_planner_ompl")

#aikido_add_test(test_OMPLSimplifier test_OMPLSimplifier.cpp)
#target_link_libraries(test_OMPLSimplifier "${PROJECT_NAME}_planner_ompl")
#
#aikido_add_test(test_TrajectoryConversions test_TrajectoryConversions.cpp)
#target_link_libraries(test_TrajectoryConversions "${PROJECT_NAME}_planner_ompl")
#
#aikido_add_test(test_StateSampler test_StateSampler.cpp)
#target_link_libraries(test_StateSampler "${PROJECT_NAME}_planner_ompl")
#
#aikido_add_test(test_ValidityChecker test_ValidityChecker.cpp)
#target_link_libraries(test_ValidityChecker "${PROJECT_NAME}_planner_ompl")

format_add_sources(OMPLTestHelpers.hpp)
