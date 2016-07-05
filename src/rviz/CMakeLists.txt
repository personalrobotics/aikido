#==============================================================================
# Dependencies
#
find_package(roscpp QUIET)
aikido_check_package(roscpp "aikido::rviz" "roscpp")
aikido_include_directories(${roscpp_INCLUDE_DIRS})

find_package(geometry_msgs QUIET)
aikido_check_package(geometry_msgs "aikido::rviz" "geometry_msgs")
aikido_include_directories(${geometry_msgs_INCLUDE_DIRS})

find_package(interactive_markers QUIET)
aikido_check_package(interactive_markers "aikido::rviz" "interactive_markers")
aikido_include_directories(${interactive_markers_INCLUDE_DIRS})

find_package(std_msgs QUIET)
aikido_check_package(std_msgs "aikido::rviz" "std_msgs")
aikido_include_directories(${std_msgs_INCLUDE_DIRS})

find_package(visualization_msgs QUIET)
aikido_check_package(visualization_msgs "aikido::rviz" "std_msgs")
aikido_include_directories(${visualization_msgs_INCLUDE_DIRS})

pkg_check_modules(libmicrohttpd QUIET libmicrohttpd)
aikido_check_package(libmicrohttpd "aikido::rviz" "libmicrohttpd")
aikido_include_directories(${libmicrohttpd_INCLUDE_DIRS})

#==============================================================================
# Libraries
#
aikido_add_library("${PROJECT_NAME}_rviz"
  shape_conversions.cpp
  BodyNodeMarker.cpp
  FrameMarker.cpp
  SkeletonMarker.cpp
  ShapeFrameMarker.cpp
  ResourceServer.cpp
  InteractiveMarkerViewer.cpp
)
target_link_libraries("${PROJECT_NAME}_rviz"
  ${DART_LIBRARIES}
  ${Boost_filesystem_LIBRARIES}
  ${libmicrohttpd_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${interactive_markers_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${libmicrohttpd_LIBRARIES}
)
