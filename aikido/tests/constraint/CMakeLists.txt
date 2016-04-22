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

aikido_add_test(test_IkSampleableConstraint
  test_IkSampleableConstraint.cpp)
target_link_libraries(test_IkSampleableConstraint
  "${PROJECT_NAME}_constraint")

# aikido_add_test(test_PolynomialConstraint
#   test_PolynomialConstraint.cpp)
# target_link_libraries(test_PolynomialConstraint
#   "${PROJECT_NAME}_constraint")

# aikido_add_test(test_TSR
#   test_TSR.cpp)
# target_link_libraries(test_TSR
#   "${PROJECT_NAME}_constraint")

aikido_add_test(test_ConjunctionConstraint
  test_ConjunctionConstraint.cpp)
target_link_libraries(test_ConjunctionConstraint
  "${PROJECT_NAME}_constraint")

# aikido_add_test(test_StackedConstraint
#   test_StackedConstraint.cpp)
# target_link_libraries(test_StackedConstraint
#   "${PROJECT_NAME}_constraint")

aikido_add_test(test_RealVectorBoxConstraint
  test_RealVectorBoxConstraint.cpp)
target_link_libraries(test_RealVectorBoxConstraint
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_distance")

aikido_add_test(test_SO2UniformSampler
  test_SO2UniformSampler.cpp)
target_link_libraries(test_SO2UniformSampler
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_distance")

aikido_add_test(test_FiniteSampleConstraint
  test_FiniteSampleConstraint.cpp)
target_link_libraries(test_FiniteSampleConstraint
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_FiniteCyclicSampleConstraint
  test_FiniteCyclicSampleConstraint.cpp)
target_link_libraries(test_FiniteCyclicSampleConstraint
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_FkTestable
  test_FkTestable.cpp)
target_link_libraries(test_FkTestable
  "${PROJECT_NAME}_constraint")

aikido_add_test(test_SO3UniformSampler
  test_SO3UniformSampler.cpp)
target_link_libraries(test_SO3UniformSampler
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_distance")

# aikido_add_test(test_SatisfiedConstraint
#   test_SatisfiedConstraint.cpp)
# target_link_libraries(test_SatisfiedConstraint
#   "${PROJECT_NAME}_constraint")

aikido_add_test(test_DartConstraintHelpers
  test_DartHelpers.cpp)
target_link_libraries(test_DartConstraintHelpers
  "${PROJECT_NAME}_constraint" "${PROJECT_NAME}_statespace")
