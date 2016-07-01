cmake_minimum_required(VERSION 2.8.8)
project(aikido_ros_control)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    geometry_msgs
    control_msgs 
    std_msgs
    trajectory_msgs
    actionlib_msgs 
    actionlib
)

find_package(DART REQUIRED COMPONENTS core)
find_package(Boost REQUIRED)
find_package(aikido REQUIRED COMPONENTS util)

include(FindPkgConfig)

include_directories(SYSTEM
  ${DART_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${aikido_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
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
    control_msgs
    trajectory_msgs
    std_msgs
    actionlib_msgs
    actionlib
  DEPENDS
    Eigen
)

include_directories(SYSTEM
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library(${PROJECT_NAME} SHARED
    src/RosTrajectoryExecutor.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})