set(sources 
  # Problems
  ConfigurationToEndEffectorOffset.cpp
  ConfigurationToEndEffectorPose.cpp
  ConfigurationToTSR.cpp
  # Planners
  ConfigurationToEndEffectorOffsetPlanner.cpp
  # ConfigurationToEndEffectorPosePlanner.cpp
  # ConfigurationToTSRPlanner.cpp
)

add_library("${PROJECT_NAME}_planner_dart" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_planner_dart"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_planner"
    ${DART_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_include_directories("${PROJECT_NAME}_planner_dart" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
)
target_compile_options("${PROJECT_NAME}_planner_dart"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_dart)
add_component_targets(${PROJECT_NAME} planner_dart "${PROJECT_NAME}_planner_dart")
add_component_dependencies(${PROJECT_NAME} planner_dart
  common
  constraint
  distance
  statespace
  trajectory
  planner
)

format_add_sources(${sources})
