aikido_add_test(test_StateSpace
  test_StateSpace.cpp)
target_link_libraries(test_StateSpace
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_MetaSkeletonStateSpace
  dart/test_MetaSkeletonStateSpace.cpp)
target_link_libraries(test_MetaSkeletonStateSpace
  "${PROJECT_NAME}_statespace")
