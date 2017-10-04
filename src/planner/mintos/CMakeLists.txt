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
    "${PROJECT_NAME}_common"
  PRIVATE
    "${PROJECT_NAME}_external_mintos"
)

add_component(${PROJECT_NAME} planner_mintos)
add_component_targets(${PROJECT_NAME} planner_mintos "${PROJECT_NAME}_planner_mintos")
add_component_dependencies(${PROJECT_NAME} planner_mintos
  constraint
  statespace
  trajectory
  common
)

format_add_sources(${sources})
