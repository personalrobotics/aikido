set(sources
  Interpolated.cpp
  Spline.cpp
  Trajectory.cpp
)

add_library("${PROJECT_NAME}_trajectory" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_trajectory"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_statespace"
)
target_compile_options("${PROJECT_NAME}_trajectory"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} trajectory)
add_component_targets(${PROJECT_NAME} trajectory "${PROJECT_NAME}_trajectory")
add_component_dependencies(${PROJECT_NAME} trajectory common statespace)

format_add_sources(${sources})
