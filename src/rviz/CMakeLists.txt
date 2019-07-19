#==============================================================================
# Dependencies
#
find_package(roscpp QUIET)
aikido_check_package(roscpp "aikido::rviz" "roscpp")

find_package(geometry_msgs QUIET)
aikido_check_package(geometry_msgs "aikido::rviz" "geometry_msgs")

# Note: Intentionally use "Module" mode because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.
find_package(interactive_markers QUIET MODULE)
aikido_check_package(interactive_markers "aikido::rviz" "interactive_markers")

find_package(std_msgs QUIET)
aikido_check_package(std_msgs "aikido::rviz" "std_msgs")

find_package(visualization_msgs QUIET)
aikido_check_package(visualization_msgs "aikido::rviz" "visualization_msgs")

pkg_check_modules(libmicrohttpd QUIET libmicrohttpd)
aikido_check_package(libmicrohttpd "aikido::rviz" "libmicrohttpd")

#==============================================================================
# Libraries
#
set(sources
  BodyNodeMarker.cpp
  FrameMarker.cpp
  InteractiveMarkerViewer.cpp
  ResourceServer.cpp
  ShapeFrameMarker.cpp
  SkeletonMarker.cpp
  shape_conversions.cpp
  TrajectoryMarker.cpp
  TSRMarker.cpp
  WorldInteractiveMarkerViewer.cpp
)

add_library("${PROJECT_NAME}_rviz" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_rviz" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
    ${interactive_markers_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${libmicrohttpd_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_rviz"
  PUBLIC
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_planner"
    ${Boost_FILESYSTEM_LIBRARY}
    ${DART_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${interactive_markers_LIBRARIES}
    ${libmicrohttpd_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${visualization_msgs_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_rviz"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} rviz)
add_component_targets(${PROJECT_NAME} rviz "${PROJECT_NAME}_rviz")
add_component_dependencies(${PROJECT_NAME} rviz
  constraint
  planner
)

# Intentionally omit tests for or_rviz. It's not clear how to test a viewer.

clang_format_add_sources(${sources})
