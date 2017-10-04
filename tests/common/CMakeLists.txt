aikido_add_test(test_Executor test_Executor.cpp)
target_link_libraries(test_Executor "${PROJECT_NAME}_common")

aikido_add_test(test_PseudoInverse test_PseudoInverse.cpp)
target_link_libraries(test_PseudoInverse "${PROJECT_NAME}_common")

aikido_add_test(test_VanDerCorput test_VanDerCorput.cpp)
target_link_libraries(test_VanDerCorput "${PROJECT_NAME}_common")

aikido_add_test(test_StepSequence test_StepSequence.cpp)
target_link_libraries(test_StepSequence "${PROJECT_NAME}_common")

aikido_add_test(test_Spline test_Spline.cpp)
target_link_libraries(test_Spline "${PROJECT_NAME}_common")

aikido_add_test(test_SplineProblem test_SplineProblem.cpp)
target_link_libraries(test_SplineProblem "${PROJECT_NAME}_common")

aikido_add_test(test_string test_string.cpp)
target_link_libraries(test_string "${PROJECT_NAME}_common")
