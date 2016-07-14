aikido_add_component(trajectory REQUIRED
  DEPENDENCIES
    statespace
    util
)
aikido_add_library("${PROJECT_NAME}_trajectory"
  COMPONENT trajectory
  SOURCES
    Interpolated.cpp
    Spline.cpp
)
target_link_libraries("${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_util"
  ${DART_LIBRARIES}
)
