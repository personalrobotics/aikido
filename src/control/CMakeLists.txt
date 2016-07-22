set(sources
  KinematicSimulationTrajectoryExecutor.cpp
  BarrettHandPositionCommandExecutor.cpp
  BarrettFingerPositionCommandExecutor.cpp
  BarrettFingerSpreadCommandExecutor.cpp
)

add_library("${PROJECT_NAME}_control" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_control"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_trajectory"
  ${DART_LIBRARIES}
)

add_component(control)
add_component_targets(control "${PROJECT_NAME}_control")
add_component_dependencies(control statespace trajectory)

coveralls_add_sources(${sources})
