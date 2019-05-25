set(sources
  KunzRetimer.cpp)

add_library("${PROJECT_NAME}_planner_kunzretimer" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_kunzretimer" SYSTEM
  PUBLIC ${DART_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_kunzretimer"
  PUBLIC
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_statespace"
    ${DART_LIBRARIES}
  PRIVATE
    "${PROJECT_NAME}_external_kunzretimer"
)
target_compile_options("${PROJECT_NAME}_planner_kunzretimer"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_kunzretimer)
add_component_targets(${PROJECT_NAME} planner_kunzretimer "${PROJECT_NAME}_planner_kunzretimer")
add_component_dependencies(${PROJECT_NAME} planner_kunzretimer
  common
  planner
  constraint
  statespace
  trajectory
)

clang_format_add_sources(${sources})
