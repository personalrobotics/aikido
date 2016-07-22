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

add_component(trajectory)
add_component_targets(trajectory "${PROJECT_NAME}_trajectory")
add_component_dependencies(trajectory statespace util)

coveralls_add_sources(${sources})
