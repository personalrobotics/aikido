#==============================================================================
# CatkinResourceRetriever
#
set(CATKIN_MAKE_WORKSPACE "${PROJECT_BINARY_DIR}/catkin_make_workspace")
file(COPY "${PROJECT_SOURCE_DIR}/tests/resources/catkin_workspace/"
  DESTINATION "${CATKIN_MAKE_WORKSPACE}")
file(WRITE "${CATKIN_MAKE_WORKSPACE}/devel/.catkin"
  "${CATKIN_MAKE_WORKSPACE}/src")

set(CATKIN_BUILD_WORKSPACE "${PROJECT_BINARY_DIR}/catkin_build_workspace")
file(COPY "${PROJECT_SOURCE_DIR}/tests/resources/catkin_workspace/"
  DESTINATION "${CATKIN_BUILD_WORKSPACE}")
file(WRITE "${PROJECT_BINARY_DIR}/catkin_build_workspace/devel/.catkin"
  "${PROJECT_BINARY_DIR}/catkin_build_workspace/src/my_package1;"
  "${PROJECT_BINARY_DIR}/catkin_build_workspace/src/my_package2")

aikido_add_test(test_CatkinResourceRetriever_catkin_make
  test_CatkinResourceRetriever.cpp)
target_link_libraries(test_CatkinResourceRetriever_catkin_make
  "${PROJECT_NAME}_io")
target_compile_definitions(test_CatkinResourceRetriever_catkin_make
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_MAKE_WORKSPACE}/devel")

aikido_add_test(test_CatkinResourceRetriever_catkin_build
  test_CatkinResourceRetriever.cpp)
target_link_libraries(test_CatkinResourceRetriever_catkin_build
  "${PROJECT_NAME}_io")
target_compile_definitions(test_CatkinResourceRetriever_catkin_build
  PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")

#==============================================================================
# Miscellaneous
#
aikido_add_test(test_KinBodyParser test_KinBodyParser.cpp)
target_link_libraries(test_KinBodyParser "${PROJECT_NAME}_io")
target_compile_definitions(test_KinBodyParser
  PRIVATE "-DAIKIDO_TEST_RESOURCES_PATH=${PROJECT_SOURCE_DIR}/tests/resources")

aikido_add_test(test_loadSkeletonFromURDF test_loadSkeletonFromURDF.cpp)
target_link_libraries(test_loadSkeletonFromURDF "${PROJECT_NAME}_io")
target_compile_definitions(test_loadSkeletonFromURDF
  PRIVATE "-DAIKIDO_TEST_RESOURCES_PATH=${PROJECT_SOURCE_DIR}/tests/resources")

aikido_add_test(test_yaml_extension test_yaml_extension.cpp)
target_link_libraries(test_yaml_extension "${PROJECT_NAME}_io")
