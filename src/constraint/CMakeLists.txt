set(sources
  uniform/RnBoxConstraint.cpp
  uniform/SO2UniformSampler.cpp
  uniform/SO3UniformSampler.cpp
  CartesianProductProjectable.cpp
  CartesianProductSampleable.cpp
  CartesianProductTestable.cpp
  CyclicSampleable.cpp
  Differentiable.cpp
  DifferentiableIntersection.cpp
  DifferentiableSubspace.cpp
  FiniteSampleable.cpp
  FrameTestable.cpp
  FrameDifferentiable.cpp
  FramePairDifferentiable.cpp
  InverseKinematicsSampleable.cpp
  JointStateSpaceHelpers.cpp
  NewtonsMethodProjectable.cpp
  NonColliding.cpp
  Projectable.cpp
  RejectionSampleable.cpp
  Sampleable.cpp
  Satisfied.cpp
  TSR.cpp
  TSRChain.cpp
  TestableIntersection.cpp
)

add_library("${PROJECT_NAME}_constraint" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_constraint"
  "${PROJECT_NAME}_statespace"
  "${PROJECT_NAME}_util"
  ${DART_LIBRARIES}
)

add_component(${PROJECT_NAME} constraint)
add_component_targets(${PROJECT_NAME} constraint "${PROJECT_NAME}_constraint")
add_component_dependencies(${PROJECT_NAME} constraint statespace util)

coveralls_add_sources(${sources})
