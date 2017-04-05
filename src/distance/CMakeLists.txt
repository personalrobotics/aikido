set(sources
  SO2Angular.cpp
  RnEuclidean.cpp
  SO3Angular.cpp
  Weighted.cpp
  defaults.cpp
)

add_library("${PROJECT_NAME}_distance" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_distance"
  PUBLIC
    "${PROJECT_NAME}_statespace"
)

add_component(${PROJECT_NAME} distance)
add_component_targets(${PROJECT_NAME} distance "${PROJECT_NAME}_distance")
add_component_dependencies(${PROJECT_NAME} distance statespace)

coveralls_add_sources(${sources})
