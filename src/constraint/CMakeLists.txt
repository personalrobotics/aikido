set(sources
  uniform/RnBoxConstraint.cpp
  uniform/RnConstantSampler.cpp
  uniform/SO2UniformSampler.cpp
  uniform/SO3UniformSampler.cpp
  uniform/SE2BoxConstraint.cpp
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
  TestableIntersection.cpp
)

add_library("${PROJECT_NAME}_constraint" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_constraint"
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_constraint"
  PUBLIC
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_util"
    ${DART_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_constraint"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} constraint)
add_component_targets(${PROJECT_NAME} constraint "${PROJECT_NAME}_constraint")
add_component_dependencies(${PROJECT_NAME} constraint statespace util)

format_add_sources(${sources})
