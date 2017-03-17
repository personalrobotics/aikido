set(sources
  Interpolated.cpp
  Spline.cpp
)

add_library("${PROJECT_NAME}_trajectory" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_util"
  ${DART_LIBRARIES}
)

add_component(${PROJECT_NAME} trajectory)
add_component_targets(${PROJECT_NAME} trajectory "${PROJECT_NAME}_trajectory")
add_component_dependencies(${PROJECT_NAME} trajectory statespace util)

coveralls_add_sources(${sources})
