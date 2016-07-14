# TODO: Only register this component if the dependencies are found.
aikido_add_component(planner_ompl
  DEPENDENCIES
    constraint
    distance
    planner
    statespace
    trajectory
)

find_package(OMPL QUIET)
aikido_check_package(OMPL "aikido::planner::ompl" "OMPL")

aikido_add_library("${PROJECT_NAME}_planner_ompl"
  COMPONENT planner_ompl
  SOURCES
    CRRT.cpp
    CRRTConnect.cpp
    dart.cpp
    GeometricStateSpace.cpp
    GoalRegion.cpp
    MotionValidator.cpp
    Planner.cpp
    StateSampler.cpp
    StateValidityChecker.cpp
)
target_link_libraries("${PROJECT_NAME}_planner_ompl"
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_trajectory"
  ${DART_LIBRARIES}
  ${OMPL_LIBRARIES}
)
