# GTest setup
add_subdirectory(gtest EXCLUDE_FROM_ALL)
include_directories("${gtest_SOURCE_DIR}/include")

# Use set_property instead of set to make this work between directories.
function(add_aikido_test target_name)
  add_executable("${target_name}" EXCLUDE_FROM_ALL ${ARGN})
  target_link_libraries("${target_name}" gtest gtest_main)
  add_test("${target_name}" "${target_name}")
  set(AIKIDO_TESTS "${target_name}" ${AIKIDO_TESTS} PARENT_SCOPE)
endfunction()


# Constraint tests
add_aikido_test(test_TSR constraint/test_TSR.cpp)
target_link_libraries(test_TSR "${PROJECT_NAME}")

# OMPL Planner test
add_aikido_test(test_OMPLPlanner test_OMPLPlanner.cpp)
target_link_libraries(test_OMPLPlanner "${PROJECT_NAME}")

add_aikido_test(test_PolynomialConstraint
 constraint/test_PolynomialConstraint.cpp)
target_link_libraries(test_PolynomialConstraint "${PROJECT_NAME}")

add_aikido_test(test_DifferentiableProjector
 constraint/test_DifferentiableProjector.cpp)
target_link_libraries(test_DifferentiableProjector "${PROJECT_NAME}")

add_aikido_test(test_FrameConstraintAdaptor
 constraint/test_FrameConstraintAdaptor.cpp)
target_link_libraries(test_FrameConstraintAdaptor "${PROJECT_NAME}")

add_aikido_test(test_FramePairConstraintAdaptor
 constraint/test_FramePairConstraintAdaptor.cpp)
target_link_libraries(test_FramePairConstraintAdaptor "${PROJECT_NAME}")

#add_aikido_test(test_IKSampleable test_IKSampleable.cpp)
#target_link_libraries(test_IKSampleable "${PROJECT_NAME}")

# StateSpace tests
add_aikido_test(test_StateSpace statespace/test_StateSpace.cpp)
target_link_libraries(test_StateSpace "${PROJECT_NAME}")

add_aikido_test(test_DartStateSpace statespace/test_DartStateSpace.cpp)
target_link_libraries(test_DartStateSpace "${PROJECT_NAME}" ${DART_LIBRARIES})

# VanDerCorput tests
add_aikido_test(test_VanDerCorput test_VanDerCorput.cpp)
target_link_libraries(test_VanDerCorput "${PROJECT_NAME}")

# SnapPlanner tests
add_aikido_test(test_SnapPlanner test_SnapPlanner.cpp)
target_link_libraries(test_SnapPlanner "${PROJECT_NAME}")

# CatkinResourceRetriever tests
set(CATKIN_MAKE_WORKSPACE
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_make_workspace")
file(COPY "../test_resources/catkin_workspace/"
  DESTINATION "${CATKIN_MAKE_WORKSPACE}")
file(WRITE "${CATKIN_MAKE_WORKSPACE}/devel/.catkin"
  "${CATKIN_MAKE_WORKSPACE}/src")

add_aikido_test(test_CatkinResourceRetriever_catkin_make
  test_CatkinResourceRetriever.cpp)
target_link_libraries(test_CatkinResourceRetriever_catkin_make
  "${PROJECT_NAME}")
target_compile_definitions(test_CatkinResourceRetriever_catkin_make
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_MAKE_WORKSPACE}/devel")

set(CATKIN_BUILD_WORKSPACE
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_build_workspace")
file(COPY "../test_resources/catkin_workspace/"
  DESTINATION "${CATKIN_BUILD_WORKSPACE}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/catkin_build_workspace/devel/.catkin"
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_build_workspace/src/my_package1;"
  "${CMAKE_CURRENT_BINARY_DIR}/catkin_build_workspace/src/my_package2")

add_aikido_test(test_CatkinResourceRetriever_catkin_build
  test_CatkinResourceRetriever.cpp)
target_link_libraries(test_CatkinResourceRetriever_catkin_build
  "${PROJECT_NAME}")
target_compile_definitions(test_CatkinResourceRetriever_catkin_build
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")

# Targets to mimic a Catkin package. "tests" builds tests and "test" (or "run_tests") runs them.
add_custom_target(tests DEPENDS ${AIKIDO_TESTS})
add_custom_target(run_tests COMMAND "${CMAKE_CTEST_COMMAND}")
