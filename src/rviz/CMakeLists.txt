#==============================================================================
# Dependencies
#
find_package(roscpp QUIET)
aikido_check_package(roscpp "rviz" "roscpp")

find_package(geometry_msgs QUIET)
aikido_check_package(geometry_msgs "rviz" "geometry_msgs")

# TODO: Workaround because find_package(interactive_markers) imports "tests"
# and "run_tests" targets that conflict with the targets defined by Aikido.
find_path(interactive_markers_INCLUDE_DIRS interactive_markers
  PATHS "/opt/ros/indigo/include"
  PATH_SUFFIXES "interactive_markeres/interactive_marker_client.h")
find_library(interactive_markers_LIBRARIES interactive_markers
  PATHS "/opt/ros/indigo/lib")
evaluate_condition(interactive_markers_FOUND
  interactive_markers_INCLUDE_DIRS AND interactive_markers_LIBRARIES)
aikido_check_package(interactive_markers "rviz" "interactive_markers")

find_package(std_msgs QUIET)
aikido_check_package(std_msgs "rviz" "std_msgs")

find_package(visualization_msgs QUIET)
aikido_check_package(visualization_msgs "rviz" "std_msgs")

pkg_check_modules(libmicrohttpd QUIET libmicrohttpd)
aikido_check_package(libmicrohttpd "rviz" "libmicrohttpd")

#==============================================================================
# Libraries
#
add_library("${PROJECT_NAME}_rviz" SHARED
  BodyNodeMarker.cpp
  FrameMarker.cpp
  InteractiveMarkerViewer.cpp
  ResourceServer.cpp
  ShapeFrameMarker.cpp
  SkeletonMarker.cpp
  shape_conversions.cpp
)
target_include_directories("${PROJECT_NAME}_rviz" SYSTEM
  PUBLIC
    ${interactive_markers_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${libmicrohttpd_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_rviz"
  ${Boost_filesystem_LIBRARIES}
  ${DART_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${interactive_markers_LIBRARIES}
  ${libmicrohttpd_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
)

add_component(${PROJECT_NAME} rviz)
add_component_targets(${PROJECT_NAME} rviz "${PROJECT_NAME}_rviz")

# Intentionally omit tests for or_rviz. It's not clear how to test a viewer.
