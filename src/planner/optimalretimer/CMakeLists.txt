set(sources
  OptimalRetimer.cpp)

add_library("${PROJECT_NAME}_planner_optimalretimer" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_optimalretimer" SYSTEM
  PUBLIC ${DART_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_optimalretimer"
  PUBLIC
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_statespace"
    ${DART_LIBRARIES}
  PRIVATE
    "${PROJECT_NAME}_external_optimaltimepathfollowing"
)
target_compile_options("${PROJECT_NAME}_planner_optimalretimer"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_optimalretimer)
add_component_targets(${PROJECT_NAME} planner_optimalretimer "${PROJECT_NAME}_planner_optimalretimer")
add_component_dependencies(${PROJECT_NAME} planner_optimalretimer
  common
  planner
  constraint
  statespace
  trajectory
)

format_add_sources(${sources})
