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

add_component(${PROJECT_NAME} control)
add_component_targets(${PROJECT_NAME} control "${PROJECT_NAME}_control")
add_component_dependencies(${PROJECT_NAME} control statespace trajectory)

coveralls_add_sources(${sources})

add_subdirectory("ros") # Dependencies: control, statespace, trajectory
