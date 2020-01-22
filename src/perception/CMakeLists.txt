#==============================================================================
# Dependencies
#
if(NOT DART_utils-urdf_FOUND)
  message(STATUS "Looking for DART component 'dart-utils-urdf' - NOT found, "
      "to use perception, please install dart-utils-urdf")
  return()
endif()

find_package(roscpp QUIET)
aikido_check_package(roscpp "aikido::perception" "roscpp")

find_package(geometry_msgs QUIET)
aikido_check_package(geometry_msgs "aikido::perception" "geometry_msgs")

find_package(std_msgs QUIET)
aikido_check_package(std_msgs "aikido::perception" "std_msgs")

find_package(visualization_msgs QUIET)
aikido_check_package(visualization_msgs "aikido::perception" "visualization_msgs")

# Note: Intentionally use "Module" mode because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.
find_package(tf QUIET MODULE)
aikido_check_package(tf "aikido::perception" "tf")

# Note: Intentionally use "Module" mode because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.
find_package(octomap_ros QUIET MODULE)
aikido_check_package(octomap_ros "aikido::perception" "octomap_ros")

find_package(pcl_conversions QUIET)
find_package(PCL REQUIRED)
aikido_check_package(PCL "aikido::perception" "PCL")

if(NOT DART_HAS_VOXELGRIDSHAPE)
    message(WARNING "The installed version of DART ${DART_VERSION} is "
      "missing dart::dynamics::VoxelGridShape, which is because DART built "
      "without octomap. Please rebuild DART from source with octomap installed "
      "to enable VoxelGridModule in AIKIDO. If you installed DART from the PPA "
      "on Ubuntu Trusty (14.04 LTS), you need to build DART from source. This "
      "is because octomap is absent on Trusty."
    )
endif()

#==============================================================================
# Libraries
#
set(sources
  PoseEstimatorModule.cpp
  shape_conversions.cpp
  AssetDatabase.cpp
  VoxelGridModule.cpp
  DetectedObject.cpp
)

add_library("${PROJECT_NAME}_perception" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_perception" SYSTEM
  PUBLIC
    ${Boost_INCLUDE_DIRS}
    ${DART_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${tf_INCLUDE_DIRS}
  PRIVATE
    ${visualization_msgs_INCLUDE_DIRS}
    ${octomap_ros_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_perception"
  PUBLIC
    "${PROJECT_NAME}_io"
    ${DART_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${tf_LIBRARIES}
  PRIVATE
    ${visualization_msgs_LIBRARIES}
    ${octomap_ros_LIBRARIES}
    ${PCL_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_perception"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)
if(DART_HAS_VOXELGRIDSHAPE)
  target_compile_definitions("${PROJECT_NAME}_perception"
    PUBLIC DART_HAS_VOXELGRIDSHAPE
  )
endif()
target_compile_definitions("${PROJECT_NAME}_perception"
  PUBLIC AIKIDO_HAS_PERCEPTION
)

add_component(${PROJECT_NAME} perception)
add_component_targets(${PROJECT_NAME} perception "${PROJECT_NAME}_perception")
add_component_dependencies(${PROJECT_NAME} perception io)

clang_format_add_sources(${sources})
