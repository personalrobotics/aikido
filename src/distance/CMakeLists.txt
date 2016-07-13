aikido_add_component(distance REQUIRED DEPENDENCIES statespace)
aikido_add_library("${PROJECT_NAME}_distance" COMPONENT distance
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
