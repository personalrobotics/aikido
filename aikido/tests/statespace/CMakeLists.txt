#############################
# StateSpace tests
#############################

add_executable(test_StateSpace test_StateSpace.cpp)
target_link_libraries(test_StateSpace "${PROJECT_NAME}" gtest gtest_main)
target_compile_definitions(test_StateSpace
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
add_test(test_StateSpace test_StateSpace)