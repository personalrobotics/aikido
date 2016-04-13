aikido_add_test(test_EuclideanDistance
  test_EuclideanDistance.cpp)
target_link_libraries(test_EuclideanDistance
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_AngularDistance
  test_AngularDistance.cpp)
target_link_libraries(test_AngularDistance
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

aikido_add_test(test_GeodesicDistance
  test_GeodesicDistance.cpp)
target_link_libraries(test_GeodesicDistance
  "${PROJECT_NAME}_distance"
  "${PROJECT_NAME}_statespace")

# aikido_add_test(test_WeightedDistance
#   test_WeightedDistance.cpp)
# target_link_libraries(test_WeightedDistance
#   "${PROJECT_NAME}_distance")
