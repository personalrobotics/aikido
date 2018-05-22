aikido_add_test(test_RnEuclidean test_RnEuclidean.cpp)
target_link_libraries(test_RnEuclidean
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_SO2Angular test_SO2Angular.cpp)
target_link_libraries(test_SO2Angular
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_SO3Angular test_SO3Angular.cpp)
target_link_libraries(test_SO3Angular
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_SE2Weighted test_SE2Weighted.cpp)
target_link_libraries(test_SE2Weighted
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_CartesianProductWeighted test_CartesianProductWeighted.cpp)
target_link_libraries(test_CartesianProductWeighted
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_DistanceMetricDefaults test_DistanceMetricDefaults.cpp)
target_link_libraries(test_DistanceMetricDefaults
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_RandomConfigurationRanker
  test_RandomConfigurationRanker.cpp)
target_link_libraries(test_RandomConfigurationRanker
    "${PROJECT_NAME}_distance")

aikido_add_test(test_NominalConfigurationRanker
  test_NominalConfigurationRanker.cpp)
target_link_libraries(test_NominalConfigurationRanker
  "${PROJECT_NAME}_distance")
