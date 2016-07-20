#open_component(perception)
#
##==============================================================================
## Dependencies
##
#pkg_check_modules(YAMLCPP QUIET yaml-cpp)
#aikido_check_package(YAMLCPP "aikido::perception" "yaml-cpp")
#include_directories(SYSTEM ${YAMLCPP_INCLUDE_DIRS})
#
#find_package(geometry_msgs QUIET)
#aikido_check_package(geometry_msgs "aikido::perception" "geometry_msgs")
#include_directories(SYSTEM ${geometry_msgs_INCLUDE_DIRS})
#
#find_package(roscpp QUIET)
#aikido_check_package(roscpp "aikido::perception" "roscpp")
#include_directories(SYSTEM ${roscpp_INCLUDE_DIRS})
#
#find_package(std_msgs QUIET)
#aikido_check_package(std_msgs "aikido::perception" "std_msgs")
#include_directories(SYSTEM ${std_msgs_INCLUDE_DIRS})
#
#find_package(visualization_msgs QUIET)
#aikido_check_package(visualization_msgs "aikido::perception"
#  "visualization_msgs")
#include_directories(SYSTEM ${visualization_msgs_INCLUDE_DIRS})
#
#find_package(tf QUIET)
#aikido_check_package(tf "aikido::perception" "tf")
#include_directories(SYSTEM ${tf_INCLUDE_DIRS})
#
##==============================================================================
## Libraries
##
#add_component_library("${PROJECT_NAME}_perception"
#  SOURCES
#    AprilTagsModule.cpp
#    shape_conversions.cpp
#    YamlAprilTagsDatabase.cpp
#)
#target_link_libraries("${PROJECT_NAME}_perception"
#  ${DART_LIBRARIES}
#  ${YAMLCPP_LIBRARIES}
#  ${geometry_msgs_LIBRARIES}
#  ${roscpp_LIBRARIES}
#  ${std_msgs_LIBRARIES}
#  ${visualization_msgs_LIBRARIES}
#  ${tf_LIBRARIES}
#)
#
#close_component(perception)
