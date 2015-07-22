cmake_minimum_required(VERSION 2.8)
project(r3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(R3_MODULE_NAME "${PROJECT_NAME}py")
add_definitions(-DR3_MODULE_NAME=${R3_MODULE_NAME})

find_package(Boost REQUIRED COMPONENTS python)
find_package(DART REQUIRED)
find_package(OMPL REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(
  include
  src
)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/ompl/plan.cpp
  src/ompl/DARTGeometricStateSpace.cpp
  src/ompl/DARTGeometricStateValidityChecker.cpp
  src/path/GeometricPath.cpp
  src/path/HauserSmoother.cpp
  src/path/SplineTrajectory.cpp
  src/path/BlendTrajectory.cpp
  src/path/ShiftTrajectory.cpp
  src/external/hauser_parabolic_smoother/DynamicPath.cpp
  src/external/hauser_parabolic_smoother/Timer.cpp
  src/external/hauser_parabolic_smoother/ParabolicRamp.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_executable(test_SplineTrajectory
  tests/test_SplineTrajectory.cpp
)
target_link_libraries(test_SplineTrajectory
  ${PROJECT_NAME}
)

#add_library(${R3_MODULE_NAME} SHARED
#  src/python/python.cpp
#  src/python/python_OMPL.cpp
#)
#target_link_libraries(${R3_MODULE_NAME}
#  ${PROJECT_NAME}
#  ${DART_LIBRARIES}
#  ${Boost_python_LIBRARIES}
#  ${PYTHON_LIBRARIES}
#  boost_numpy
#  boost_numpy_eigen
#)
#set_target_properties(${R3_MODULE_NAME} PROPERTIES
#  PREFIX ""
#  SUFFIX ".so"
#)

# Installation
# TODO: Also install the Python module.
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS test_SplineTrajectory DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

# This is required by REP-136.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
