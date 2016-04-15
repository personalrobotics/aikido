aikido_add_library("${PROJECT_NAME}_path"
  PiecewiseLinearTrajectory.cpp
)
target_link_libraries("${PROJECT_NAME}_path"
  "${PROJECT_NAME}_statespace"
  ${DART_LIBRARIES}
)
