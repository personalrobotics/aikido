################################
# GTest setup
################################
add_subdirectory(gtest)
include_directories(${gtest_SOURCE_DIR}/include)

#############################
# SampleableConstraint tests
#############################
add_executable(test_TSR EXCLUDE_FROM_ALL test_TSR.cpp)
target_link_libraries(test_TSR "${PROJECT_NAME}" gtest gtest_main)
target_compile_definitions(test_TSR
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
add_test(test_TSR test_TSR)

add_executable(test_IKSampleable EXCLUDE_FROM_ALL test_IKSampleable.cpp)
target_link_libraries(test_IKSampleable "${PROJECT_NAME}" gtest gtest_main)
target_compile_definitions(test_IKSampleable
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
add_test(test_IKSampleable test_IKSampleable)


################################
# CatkinResourceRetriever tests
################################
set(CATKIN_MAKE_WORKSPACE
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_make_workspace")
file(COPY "../test_resources/catkin_workspace/"
  DESTINATION "${CATKIN_MAKE_WORKSPACE}")
file(WRITE "${CATKIN_MAKE_WORKSPACE}/devel/.catkin"
  "${CATKIN_MAKE_WORKSPACE}/src")

add_executable(test_CatkinResourceRetriever_catkin_make EXCLUDE_FROM_ALL
  test_CatkinResourceRetriever.cpp)
target_link_libraries(test_CatkinResourceRetriever_catkin_make
  "${PROJECT_NAME}" gtest gtest_main)
target_compile_definitions(test_CatkinResourceRetriever_catkin_make
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_MAKE_WORKSPACE}/devel")

add_test(test_CatkinResourceRetriever_catkin_make
  test_CatkinResourceRetriever_catkin_make)

set(CATKIN_BUILD_WORKSPACE
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_build_workspace")
file(COPY "../test_resources/catkin_workspace/"
  DESTINATION "${CATKIN_BUILD_WORKSPACE}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/catkin_build_workspace/devel/.catkin"
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_build_workspace/src/my_package1;"
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_build_workspace/src/my_package2")

add_executable(test_CatkinResourceRetriever_catkin_build EXCLUDE_FROM_ALL
  test_CatkinResourceRetriever.cpp)
target_link_libraries(test_CatkinResourceRetriever_catkin_build
  "${PROJECT_NAME}" gtest gtest_main)
target_compile_definitions(test_CatkinResourceRetriever_catkin_build
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
add_test(test_CatkinResourceRetriever_catkin_build
  test_CatkinResourceRetriever_catkin_build)

# Define custom targets to match Catkin's convention.
add_custom_target(tests DEPENDS
  test_TSR
  test_IKSampleable
  test_CatkinResourceRetriever_catkin_make
  test_CatkinResourceRetriever_catkin_build
)
add_custom_target(run_tests COMMAND "${CMAKE_CTEST_COMMAND}")
