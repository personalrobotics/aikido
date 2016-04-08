
# #############################
# # SampleableConstraint tests
# #############################
# add_executable(test_TSR test_TSR.cpp)
# target_link_libraries(test_TSR "${PROJECT_NAME}" gtest gtest_main)
# target_compile_definitions(test_TSR
#   PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
# add_test(test_TSR test_TSR)

# add_executable(test_IKSampleable test_IKSampleable.cpp)
# target_link_libraries(test_IKSampleable "${PROJECT_NAME}" gtest gtest_main)
# target_compile_definitions(test_IKSampleable
#   PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
# add_test(test_IKSampleable test_IKSampleable)


# #############################
# # DifferentiableConstraint tests
# #############################

# add_executable(test_Polynomial test_PolynomialConstraint.cpp)
# target_link_libraries(test_Polynomial "${PROJECT_NAME}" gtest gtest_main)
# target_compile_definitions(test_Polynomial
#   PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
# add_test(test_Polynomial test_Polynomial)

# # add_executable(test_FKConstraint test_FKConstraint.cpp)
# # target_link_libraries(test_FKConstraint "${PROJECT_NAME}" gtest gtest_main)
# # target_compile_definitions(test_FKConstraint
# #   PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
# # add_test(test_FKConstraint test_FKConstraint)

# # add_executable(test_CompoundConstraint test_CompoundConstraint.cpp)
# # target_link_libraries(test_CompoundConstraint "${PROJECT_NAME}" gtest gtest_main)
# # target_compile_definitions(test_CompoundConstraint
# #   PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
# # add_test(test_CompoundConstraint test_CompoundConstraint)

# add_executable(test_ChainRuleAdaptor test_ChainRuleAdaptor.cpp)
# target_link_libraries(test_ChainRuleAdaptor "${PROJECT_NAME}" gtest gtest_main)
# target_compile_definitions(test_ChainRuleAdaptor
#   PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
# add_test(test_ChainRuleAdaptor test_ChainRuleAdaptor)



# #############################
# # ProjectableConstraint tests
# #############################

# add_executable(test_ProjectableByDifferentiable 
# 			   test_ProjectableByDifferentiable.cpp)
# target_link_libraries(test_ProjectableByDifferentiable "${PROJECT_NAME}"
# 					  gtest gtest_main)
# target_compile_definitions(test_Polynomial
#   PRIVATE "-DAIKIDO_TEST_WORKSPACE_PATH=${CATKIN_BUILD_WORKSPACE}/devel")
# add_test(test_ProjectableByDifferentiable test_ProjectableByDifferentiable)