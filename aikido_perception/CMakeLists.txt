cmake_minimum_required(VERSION 2.8.8)
project(aikido_perception)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  std_msgs
  visualization_msgs
)

find_package(DART REQUIRED COMPONENTS core)
find_package(Boost REQUIRED)
find_package(aikido REQUIRED COMPONENTS util)

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)
add_definitions(-DYAMLCPP_NEWAPI)

include_directories(SYSTEM
  ${DART_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

add_definitions(-DPROJECT_NAME=${PROJECT_NAME})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    tf
    std_msgs
    visualization_msgs
  DEPENDS
    Eigen
)

include_directories(SYSTEM
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${YAMLCPP_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(${PROJECT_NAME} SHARED
	src/AprilTagsModule.cpp
  src/shape_conversions.cpp
  src/YamlAprilTagsDatabase.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YamlCpp_LIBRARIES}
  ${tf_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})