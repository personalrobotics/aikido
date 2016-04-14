aikido_add_test(test_StateSpace test_StateSpace.cpp)
target_link_libraries(test_StateSpace "${PROJECT_NAME}_statespace")

aikido_add_test(test_DartStateSpace test_DartStateSpace.cpp)
target_link_libraries(test_DartStateSpace "${PROJECT_NAME}_statespace")
