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

aikido_add_test(test_WeightedDistance test_WeightedDistance.cpp)
target_link_libraries(test_WeightedDistance
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_DistanceMetricDefaults test_DistanceMetricDefaults.cpp)
target_link_libraries(test_DistanceMetricDefaults
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")
