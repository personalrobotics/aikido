set(sources
  SO2Angular.cpp
  RnEuclidean.cpp
  SO3Angular.cpp
  Weighted.cpp
  defaults.cpp
)

add_library("${PROJECT_NAME}_distance" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace"
  ${DART_LIBRARIES}
)

add_component(distance)
add_component_targets(distance "${PROJECT_NAME}_distance")
add_component_dependencies(distance statespace)

coveralls_add_sources(${sources})
