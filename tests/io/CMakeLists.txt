#==============================================================================
# Miscellaneous
#
aikido_add_test(test_KinBodyParser test_KinBodyParser.cpp)
target_link_libraries(test_KinBodyParser "${PROJECT_NAME}_io")
target_compile_definitions(test_KinBodyParser
  PRIVATE "-DAIKIDO_TEST_RESOURCES_PATH=${PROJECT_SOURCE_DIR}/tests/resources")

aikido_add_test(test_yaml_extension test_yaml_extension.cpp)
target_link_libraries(test_yaml_extension "${PROJECT_NAME}_io")
