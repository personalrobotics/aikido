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
  ${Boost_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/ompl/plan.cpp
  src/ompl/DARTGeometricStateSpace.cpp
  src/ompl/DARTGeometricStateValidityChecker.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_library(${R3_MODULE_NAME} SHARED
  src/python/python.cpp
  src/python/python_DartLoader.cpp
  src/python/python_DegreeOfFreedom.cpp
  src/python/python_OMPL.cpp
  src/python/python_SimWindow.cpp
  src/python/python_Skeleton.cpp
  src/python/python_World.cpp
)
target_link_libraries(${R3_MODULE_NAME}
  ${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
  boost_numpy
  boost_numpy_eigen
)
set_target_properties(${R3_MODULE_NAME} PROPERTIES
  PREFIX ""
  SUFFIX ".so"
)
