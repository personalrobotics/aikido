set(sources
  TrajectoryRunningException.cpp
  InstantaneousTrajectoryExecutor.cpp
  KinematicSimulationTrajectoryExecutor.cpp
  QueuedTrajectoryExecutor.cpp
  BarrettHandKinematicSimulationPositionCommandExecutor.cpp
  BarrettFingerKinematicSimulationPositionCommandExecutor.cpp
  BarrettFingerKinematicSimulationSpreadCommandExecutor.cpp
)

add_library("${PROJECT_NAME}_control" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_control"
  PUBLIC ${DART_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_control"
  PUBLIC
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_trajectory"
    ${DART_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_control"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} control)
add_component_targets(${PROJECT_NAME} control "${PROJECT_NAME}_control")
add_component_dependencies(${PROJECT_NAME} control statespace trajectory)

format_add_sources(${sources})

add_subdirectory("ros") # Dependencies: control, statespace, trajectory
