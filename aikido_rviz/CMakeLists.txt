cmake_minimum_required(VERSION 2.4.6)
project(aikido_rviz)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  interactive_markers
  std_msgs
  visualization_msgs
)

find_package(DART REQUIRED CONFIG)
find_package(Boost REQUIRED filesystem python)
find_package(PythonLibs REQUIRED)

include(FindPkgConfig)
pkg_check_modules(libmicrohttpd REQUIRED libmicrohttpd)

include_directories(SYSTEM
  ${DART_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${libmicrohttpd_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)
link_directories(
  ${DART_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${libmicrohttpd_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    interactive_markers
    std_msgs
    visualization_msgs
  DEPENDS
    Eigen
    libmicrohttpd
)

add_definitions(-DPROJECT_NAME=${PROJECT_NAME})

include_directories(SYSTEM
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

include(DetectCXX11Flags)

add_library(${PROJECT_NAME} SHARED
  src/shape_conversions.cpp
  src/BodyNodeMarker.cpp
  src/FrameMarker.cpp
  src/SkeletonMarker.cpp
  src/ShapeFrameMarker.cpp
  src/ResourceServer.cpp
  src/InteractiveMarkerViewer.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${DART_LIBRARIES}
  ${Boost_filesystem_LIBRARIES}
  ${libmicrohttpd_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_python SHARED
  src/python.cpp
)
target_link_libraries(${PROJECT_NAME}_python
  ${PROJECT_NAME}
  ${Boost_PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_python PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}"
  OUTPUT_NAME ${PROJECT_NAME}
  PREFIX ""
  SUFFIX ".so"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}_python
  LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
)
