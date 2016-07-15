include_directories(SYSTEM ${mintos_INCLUDE_DIRS})

aikido_add_library("${PROJECT_NAME}_planner_mintos" MinTOS.cpp)
target_link_libraries("${PROJECT_NAME}_planner_mintos"
  "${PROJECT_NAME}_mintos"
  "${PROJECT_NAME}_trajectory"
  "${PROJECT_NAME}_util"
  "${PROJECT_NAME}_statespace")
