set(sources 
  VectorFieldPlanner.cpp
  VectorField.cpp
  VectorFieldUtil.cpp
  detail/VectorFieldIntegrator.cpp
  detail/VectorFieldPlannerExceptions.cpp
  detail/BodyNodePoseVectorFieldConstraint.cpp
  BodyNodePoseVectorField.cpp
  MoveEndEffectorOffsetVectorField.cpp
  MoveEndEffectorPoseVectorField.cpp
  MoveEndEffectorAlongWorkspacePathVectorField.cpp
)

add_library("${PROJECT_NAME}_planner_vectorfield" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_planner_vectorfield"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_statespace"
    ${DART_LIBRARIES}
    ${Boost_LIBRARIES}
)
target_include_directories("${PROJECT_NAME}_planner_vectorfield" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
)
target_compile_options("${PROJECT_NAME}_planner_vectorfield"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_vectorfield)
add_component_targets(${PROJECT_NAME} planner_vectorfield "${PROJECT_NAME}_planner_vectorfield")
add_component_dependencies(${PROJECT_NAME} planner_vectorfield
  common
  constraint
  distance
  statespace
  trajectory
)

format_add_sources(${sources})

