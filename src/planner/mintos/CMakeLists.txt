set(sources MinTOS.cpp)

add_library("${PROJECT_NAME}_planner_mintos" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_mintos"
  PRIVATE ${mintos_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_mintos"
  PUBLIC
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_util"
  PRIVATE
    "${PROJECT_NAME}_external_mintos"
)

add_component(planner_mintos)
add_component_targets(planner_mintos "${PROJECT_NAME}_planner_mintos")
add_component_dependencies(planner_mintos
  constraint
  statespace
  trajectory
  util
)

coveralls_add_sources(${sources})
