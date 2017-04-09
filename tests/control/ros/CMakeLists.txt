if(TARGET "${PROJECT_NAME}_control_ros")
  aikido_add_test(test_Conversions
      test_Conversions.cpp)
  target_link_libraries(test_Conversions
    "${PROJECT_NAME}_control_ros")
endif()

