

#############################
# State tests
#############################

add_executable(test_State test_State.cpp)
target_link_libraries(test_State "${PROJECT_NAME}" gtest gtest_main)
target_compile_definitions(test_State
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
add_test(test_State test_State)

# add_executable(test_Jacobian test_Jacobian.cpp)
# target_link_libraries(test_State "${PROJECT_NAME}" gtest gtest_main)
# target_compile_definitions(test_Jacobian
#   PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
# add_test(test_Jacobian test_Jacobian)