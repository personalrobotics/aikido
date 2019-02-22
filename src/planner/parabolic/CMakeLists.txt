set(sources
  ParabolicTimer.cpp
  ParabolicSmoother.cpp
  ParabolicUtil.cpp
  HauserParabolicSmootherHelpers.cpp)

add_library("${PROJECT_NAME}_planner_parabolic" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_parabolic" SYSTEM
  PUBLIC ${DART_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_parabolic"
  PUBLIC
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_statespace"
    ${DART_LIBRARIES}
  PRIVATE
    "${PROJECT_NAME}_external_hauserparabolicsmoother"
)
target_compile_options("${PROJECT_NAME}_planner_parabolic"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_parabolic)
add_component_targets(${PROJECT_NAME} planner_parabolic "${PROJECT_NAME}_planner_parabolic")
add_component_dependencies(${PROJECT_NAME} planner_parabolic
  common
  planner
  constraint
  statespace
  trajectory
)

clang_format_add_sources(${sources})
