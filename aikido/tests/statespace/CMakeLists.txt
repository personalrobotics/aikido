aikido_add_test(test_RealVectorStateSpace test_RealVectorStateSpace.cpp)
target_link_libraries(test_RealVectorStateSpace "${PROJECT_NAME}_statespace")

aikido_add_test(test_SO2StateSpace test_SO2StateSpace.cpp)
target_link_libraries(test_SO2StateSpace "${PROJECT_NAME}_statespace")

aikido_add_test(test_SO3StateSpace test_SO3StateSpace.cpp)
target_link_libraries(test_SO3StateSpace "${PROJECT_NAME}_statespace")

aikido_add_test(test_SE2StateSpace test_SE2StateSpace.cpp)
target_link_libraries(test_SE2StateSpace "${PROJECT_NAME}_statespace")

aikido_add_test(test_SE3StateSpace test_SE3StateSpace.cpp)
target_link_libraries(test_SE3StateSpace "${PROJECT_NAME}_statespace")

aikido_add_test(test_CompoundStateSpace test_CompoundStateSpace.cpp)
target_link_libraries(test_CompoundStateSpace "${PROJECT_NAME}_statespace")

aikido_add_test(test_MetaSkeletonStateSpace
  dart/test_MetaSkeletonStateSpace.cpp)
target_link_libraries(test_MetaSkeletonStateSpace
  "${PROJECT_NAME}_statespace")
