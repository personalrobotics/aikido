if(OMPL_VERSION VERSION_LESS 0.14.0)
  message(STATUS "OMPL Planners are disabled for versions < 0.14.0")
  return()
endif()

#==============================================================================
# Libraries
#
set(sources 
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

add_library("${PROJECT_NAME}_planner_ompl" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_ompl" SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_ompl"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_planner"
    ${DART_LIBRARIES}
    ${OMPL_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_planner_ompl"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_ompl)
add_component_targets(${PROJECT_NAME} planner_ompl "${PROJECT_NAME}_planner_ompl")
add_component_dependencies(${PROJECT_NAME} planner_ompl
  constraint
  distance
  planner
  statespace
  trajectory
)

format_add_sources(${sources})
