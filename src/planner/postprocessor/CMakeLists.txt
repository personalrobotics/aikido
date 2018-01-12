set(sources
  postprocess.cpp
  RetimeTrajectoryPostProcessor.cpp
  SmoothTrajectoryPostProcessor.cpp)

add_library("${PROJECT_NAME}_planner_postprocessor" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_postprocessor" SYSTEM
  PUBLIC ${DART_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_postprocessor"
  PUBLIC
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_statespace"
    ${DART_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_planner_postprocessor"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_postprocessor)
add_component_targets(${PROJECT_NAME} planner_postprocessor "${PROJECT_NAME}_planner_postprocessor")
add_component_dependencies(${PROJECT_NAME} planner_postprocessor
  common
  planner
  constraint
  statespace
  trajectory
)

format_add_sources(${sources})
