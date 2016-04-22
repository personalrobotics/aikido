aikido_add_test(test_DifferentiableProjector 
  PolynomialConstraint.cpp
  test_DifferentiableProjector.cpp)
target_link_libraries(test_DifferentiableProjector
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_DifferentiableSubSpace
  PolynomialConstraint.cpp
  test_DifferentiableSubSpace.cpp)
target_link_libraries(test_DifferentiableSubSpace
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_FrameConstraintAdaptor
  test_FrameConstraintAdaptor.cpp)
target_link_libraries(test_FrameConstraintAdaptor
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_FramePairConstraintAdaptor
  test_FramePairConstraintAdaptor.cpp)
target_link_libraries(test_FramePairConstraintAdaptor
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_InverseKinematicsSampleable
  test_InverseKinematicsSampleable.cpp)
target_link_libraries(test_InverseKinematicsSampleable
  "${PROJECT_NAME}_constraint")

# aikido_add_test(test_PolynomialConstraint
#   test_PolynomialConstraint.cpp)
# target_link_libraries(test_PolynomialConstraint
#   "${PROJECT_NAME}_constraint")

# aikido_add_test(test_TSR
#   test_TSR.cpp)
# target_link_libraries(test_TSR
#   "${PROJECT_NAME}_constraint")

aikido_add_test(test_TestableIntersection
  test_TestableIntersection.cpp)
target_link_libraries(test_TestableIntersection
  "${PROJECT_NAME}_constraint")

# aikido_add_test(test_DifferentiableIntersection
#   test_DifferentiableIntersection.cpp)
# target_link_libraries(test_DifferentiableIntersection
#   "${PROJECT_NAME}_constraint")

aikido_add_test(test_RealVectorBoxConstraint
  test_RealVectorBoxConstraint.cpp)
target_link_libraries(test_RealVectorBoxConstraint
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

aikido_add_test(test_FkTestable
  test_FkTestable.cpp)
target_link_libraries(test_FkTestable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_SO3UniformSampler
  test_SO3UniformSampler.cpp)
target_link_libraries(test_SO3UniformSampler
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_distance")

# aikido_add_test(test_Satisfied
#   test_Satisfied.cpp)
# target_link_libraries(test_Satisfied
#   "${PROJECT_NAME}_constraint")

aikido_add_test(test_DartConstraintHelpers
  test_DartHelpers.cpp)
target_link_libraries(test_DartConstraintHelpers
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_statespace")
