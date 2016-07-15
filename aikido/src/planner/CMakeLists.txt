add_subdirectory("ompl")
add_subdirectory("parabolic")
add_subdirectory("mintos")

aikido_add_library("${PROJECT_NAME}_planner" SnapPlanner.cpp)
target_link_libraries("${PROJECT_NAME}_planner"
  "${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_util")
