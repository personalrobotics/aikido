#==============================================================================
# Libraries
#
set(sources 
  OptimizationInterpolated.cpp
  Function.cpp
  MetaSkeletonTrajectoryVariable.cpp
  Planner.cpp
  SplineCoefficientsVariable.cpp
  SplineCoefficientsAndDurationsVariable.cpp
  SplineDurationsVariable.cpp
  SplineVariable.cpp
  TrajectoryVariable.cpp
  Variable.cpp
)

add_library("${PROJECT_NAME}_planner_optimization" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_optimization" SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_optimization"
  PUBLIC
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_planner"
    ${DART_LIBRARIES}
    ${OMPL_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_planner_optimization"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_optimization)
add_component_targets(${PROJECT_NAME} planner_optimization
  "${PROJECT_NAME}_planner_optimization")
add_component_dependencies(${PROJECT_NAME} planner_optimization
  constraint
  distance
  planner
  statespace
  trajectory
)

format_add_sources(${sources})
