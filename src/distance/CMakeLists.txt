add_library("${PROJECT_NAME}_distance" SHARED
  SO2Angular.cpp
  RnEuclidean.cpp
  SO3Angular.cpp
  Weighted.cpp
  defaults.cpp
)
target_link_libraries("${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace"
  ${DART_LIBRARIES}
)

add_component(distance)
add_component_targets(distance "${PROJECT_NAME}_distance")
add_component_dependencies(distance statespace)
