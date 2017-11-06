#==============================================================================
# Dependencies
#
find_package(roscpp QUIET)
aikido_check_package(roscpp "aikido::sensor" "roscpp")

find_package(cv_bridge)

#find_package(catkin REQUIRED)
find_package(eyerectoo_stream)
#catkin_package(CATKIN_DEPENDS eyerectoo_stream)
#add_message_files(FILES Journal.msg)

# TODO: What is the correct way of including OpenCV?
# OpenCV deps
include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)

#==============================================================================
# Libraries
#
set(sources
  EyeTracker.cpp
)

#add_library("${PROJECT_NAME}_sensor" SHARED ${sources})
# TODO: This is just a hack to actually run the subscriber.
add_executable("${PROJECT_NAME}_sensor" ${sources})
target_include_directories("${PROJECT_NAME}_sensor" SYSTEM
  PUBLIC
    ${roscpp_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIRS}
)

target_link_libraries("${PROJECT_NAME}_sensor"
  PUBLIC
  ${DART_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OPENCV_LIBRARIES}
)

# TODO: This is just a hack to actually run the subscriber.
#add_component(${PROJECT_NAME} sensor)
#add_component_targets(${PROJECT_NAME} sensor "${PROJECT_NAME}_sensor")

format_add_sources(${sources})

target_compile_options("${PROJECT_NAME}_sensor"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)


