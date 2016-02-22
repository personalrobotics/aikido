################################
# GTest setup
################################
add_subdirectory(gtest)
include_directories(${gtest_SOURCE_DIR}/include)

function(add_aikido_test source)
  get_filename_component(test_name ${source} NAME_WE)

  set(test_libraries ${ARGV})
  list(REMOVE_AT test_libraries 0)

  add_executable("${test_name}" "${source}")
  target_link_libraries("${test_name}" ${test_libraries})
  add_test("${test_name}" "${test_name}")
endfunction()

################################
# CatkinResourceRetriever setup
################################
set(CATKIN_MAKE_WORKSPACE
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_make_workspace")
file(COPY "../test_resources/catkin_workspace/"
  DESTINATION "${CATKIN_MAKE_WORKSPACE}")
file(WRITE "${CATKIN_MAKE_WORKSPACE}/devel/.catkin"
  "${CATKIN_MAKE_WORKSPACE}/src")

add_executable(test_CatkinResourceRetriever_catkin_make
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

add_executable(test_CatkinResourceRetriever_catkin_build
  test_CatkinResourceRetriever.cpp)
target_link_libraries(test_CatkinResourceRetriever_catkin_build
  "${PROJECT_NAME}" gtest gtest_main)
target_compile_definitions(test_CatkinResourceRetriever_catkin_build
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
add_test(test_CatkinResourceRetriever_catkin_build
  test_CatkinResourceRetriever_catkin_build)

################################
# Unit test declarations
################################
add_aikido_test(test_TSR.cpp aikido gtest gtest_main)
add_aikido_test(test_Spline.cpp aikido gtest gtest_main)
add_aikido_test(test_SplineProblem.cpp aikido gtest gtest_main)
