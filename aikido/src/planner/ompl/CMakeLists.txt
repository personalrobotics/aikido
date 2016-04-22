include_directories(SYSTEM ${OMPL_INCLUDE_DIRS})

aikido_add_library("${PROJECT_NAME}_planner_ompl"
  AIKIDOGeometricStateSpace.cpp
  AIKIDOStateSampler.cpp
  AIKIDOStateValidityChecker.cpp
  OMPLPlanner.cpp
  GoalRegion.cpp
)
target_link_libraries("${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_trajectory"
  ${Boost_LIBRARIES}
  ${DART_LIBRARIES}
  ${OMPL_LIBRARIES}
)
