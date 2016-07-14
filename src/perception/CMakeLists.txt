aikido_add_component(perception)

#==============================================================================
# Dependencies
#
pkg_check_modules(YAMLCPP QUIET yaml-cpp)
aikido_check_package(YAMLCPP "aikido::perception" "yaml-cpp")
aikido_include_directories(COMPONENT perception ${YAMLCPP_INCLUDE_DIRS})

find_package(geometry_msgs QUIET)
aikido_check_package(geometry_msgs "aikido::perception" "geometry_msgs")
aikido_include_directories(COMPONENT perception ${geometry_msgs_INCLUDE_DIRS})

find_package(roscpp QUIET)
aikido_check_package(roscpp "aikido::perception" "roscpp")
aikido_include_directories(COMPONENT perception ${roscpp_INCLUDE_DIRS})

find_package(std_msgs QUIET)
aikido_check_package(std_msgs "aikido::perception" "std_msgs")
aikido_include_directories(COMPONENT perception ${std_msgs_INCLUDE_DIRS})

find_package(visualization_msgs QUIET)
aikido_check_package(visualization_msgs "aikido::perception"
  "visualization_msgs")
aikido_include_directories(COMPONENT perception
  ${visualization_msgs_INCLUDE_DIRS}
)

find_package(tf QUIET)
aikido_check_package(tf "aikido::perception" "tf")
aikido_include_directories(COMPONENT perception ${tf_INCLUDE_DIRS})

#==============================================================================
# Libraries
#
aikido_add_library("${PROJECT_NAME}_perception"
  SOURCES
    AprilTagsModule.cpp
    shape_conversions.cpp
    YamlAprilTagsDatabase.cpp
)
target_link_libraries("${PROJECT_NAME}_perception"
  ${DART_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${tf_LIBRARIES}
)
