aikido_add_test(test_Rn test_Rn.cpp)
target_link_libraries(test_Rn "${PROJECT_NAME}_statespace")

aikido_add_test(test_SO2 test_SO2.cpp)
target_link_libraries(test_SO2 "${PROJECT_NAME}_statespace")

aikido_add_test(test_SO3 test_SO3.cpp)
target_link_libraries(test_SO3 "${PROJECT_NAME}_statespace")

aikido_add_test(test_SE2 test_SE2.cpp)
target_link_libraries(test_SE2 "${PROJECT_NAME}_statespace")

aikido_add_test(test_SE3 test_SE3.cpp)
target_link_libraries(test_SE3 "${PROJECT_NAME}_statespace")

aikido_add_test(test_CartesianProduct test_CartesianProduct.cpp)
target_link_libraries(test_CartesianProduct "${PROJECT_NAME}_statespace")

aikido_add_test(test_MetaSkeletonStateSpace
  dart/test_MetaSkeletonStateSpace.cpp)
target_link_libraries(test_MetaSkeletonStateSpace
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_DartJointStateSpaces dart/test_DartJointStateSpaces.cpp)
target_link_libraries(test_DartJointStateSpaces "${PROJECT_NAME}_statespace")
