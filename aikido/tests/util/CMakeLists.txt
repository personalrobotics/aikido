#############################
# State tests
#############################

add_executable(test_PseudoInverse test_PseudoInverse.cpp)
target_link_libraries(test_PseudoInverse "${PROJECT_NAME}" gtest gtest_main)
target_compile_definitions(test_PseudoInverse
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
add_test(test_PseudoInverse test_PseudoInverse)