option(AIKIDO_USE_TRAJECTORY "Enable Aikido Trajectory component." ON)
if(NOT ${AIKIDO_USE_TRAJECTORY})
  return()
endif()
add_library("${PROJECT_NAME}_trajectory" SHARED
  Interpolated.cpp
  Spline.cpp
)
target_link_libraries("${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_util"
  ${DART_LIBRARIES}
)

add_component(trajectory)
add_component_targets(trajectory "${PROJECT_NAME}_trajectory")
add_component_dependencies(trajectory statespace util)
