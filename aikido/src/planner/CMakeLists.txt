aikido_add_library("${PROJECT_NAME}_planner"
  VectorFieldPlanner.cpp
)
target_link_libraries("${PROJECT_NAME}_planner"
  ${DART_LIBRARIES}
)
