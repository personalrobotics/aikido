aikido_add_test(test_NewtonsMethodProjectable 
  PolynomialConstraint.cpp
  test_NewtonsMethodProjectable.cpp)
target_link_libraries(test_NewtonsMethodProjectable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_DifferentiableSubSpace
  PolynomialConstraint.cpp
  test_DifferentiableSubSpace.cpp)
target_link_libraries(test_DifferentiableSubSpace
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_FrameDifferentiable
  test_FrameDifferentiable.cpp)
target_link_libraries(test_FrameDifferentiable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_FramePairDifferentiable
  test_FramePairDifferentiable.cpp)
target_link_libraries(test_FramePairDifferentiable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_InverseKinematicsSampleable
  test_InverseKinematicsSampleable.cpp)
target_link_libraries(test_InverseKinematicsSampleable
  "${PROJECT_NAME}_constraint")

# aikido_add_test(test_PolynomialConstraint
#   test_PolynomialConstraint.cpp)
# target_link_libraries(test_PolynomialConstraint
#   "${PROJECT_NAME}_constraint")

aikido_add_test(test_TSR
	PolynomialConstraint.cpp
  test_TSR.cpp)
target_link_libraries(test_TSR
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_TestableIntersection
  test_TestableIntersection.cpp)
target_link_libraries(test_TestableIntersection
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_DifferentiableIntersection
	PolynomialConstraint.cpp
  test_DifferentiableIntersection.cpp)
target_link_libraries(test_DifferentiableIntersection
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_RnBoxConstraint
  test_RnBoxConstraint.cpp)
target_link_libraries(test_RnBoxConstraint
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_distance")

aikido_add_test(test_SO2UniformSampler
  test_SO2UniformSampler.cpp)
target_link_libraries(test_SO2UniformSampler
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_distance")

aikido_add_test(test_FiniteSampleable
  test_FiniteSampleable.cpp)
target_link_libraries(test_FiniteSampleable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_CyclicSampleable
  test_CyclicSampleable.cpp)
target_link_libraries(test_CyclicSampleable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_FrameTestable
  test_FrameTestable.cpp)
target_link_libraries(test_FrameTestable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_SO3UniformSampler
  test_SO3UniformSampler.cpp)
target_link_libraries(test_SO3UniformSampler
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_distance")

aikido_add_test(test_Satisfied
  test_Satisfied.cpp)
target_link_libraries(test_Satisfied
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_DartConstraintHelpers
  test_DartHelpers.cpp)
target_link_libraries(test_DartConstraintHelpers
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_statespace")

aikido_add_test(test_TestableSubspace
  test_TestableSubspace.cpp)
target_link_libraries(test_TestableSubspace
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_SampleableSubSpace
  test_SampleableSubSpace.cpp)
target_link_libraries(test_SampleableSubSpace
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_CartesianProductProjectable
  test_CartesianProductProjectable.cpp)
target_link_libraries(test_CartesianProductProjectable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_Projectable
  test_Projectable.cpp)
target_link_libraries(test_Projectable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_NonColliding
  test_NonColliding.cpp)
target_link_libraries(test_NonColliding
  "${PROJECT_NAME}_constraint")
