# Find OMPL
#
# We enforce to use the custom OMPL package provided by PRL PPA
# (https://launchpad.net/~personalrobotics/+archive/ubuntu/ppa) instead of
# ros-kinetic-ompl, which has an issue with AIKIDO. See
# https://github.com/personalrobotics/aikido/issues/363 for the details.
if("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
  # Search system path over ROS package path to find libompl-dev instead of
  # ros-kinetic-ompl. We take this approach because there is no way to
  # prevent searching ROS package path (welcome to suggest better way).
  list(FIND CMAKE_PREFIX_PATH "/usr/" CMAKE_PREFIX_PATH_HAS_USR)
  if(CMAKE_PREFIX_PATH_HAS_USR EQUAL -1)
    list(APPEND CMAKE_PREFIX_PATH "/usr/")
    find_package(ompl)
    list(REMOVE_AT CMAKE_PREFIX_PATH CMAKE_PREFIX_PATH_HAS_USR)
  else()
    find_package(ompl)
  endif()
  # ros-kinetic-ompl can be found in case libompl-dev isn't installed. Skip
  # building OMPL planners in this case.
  if(OMPL_LIBRARY MATCHES "^/opt/ros/kinetic/")
    message(WARNING "It appears that CMake found ros-kinetic-ompl, "
      "which can cause segfaults with AIKIDO "
      "(see https://github.com/personalrobotics/aikido/issues/363). "
      "In order to enable OMPL planners, please install libompl-dev provided "
      "by PRL PPA as:\n"
      "  $ sudo apt-add-repository ppa:personalrobotics/ppa\n"
      "  $ sudo apt update\n"
      "  $ sudo apt install libompl-dev"
    )
    return()
  endif()
else()
  find_package(ompl)
endif()
if(OMPL_VERSION VERSION_LESS 0.14.0)
  message(STATUS "OMPL planners are disabled for versions < 0.14.0")
  return()
endif()

# Libraries
#
set(sources
  CRRT.cpp
  CRRTConnect.cpp
  dart.cpp
  GeometricStateSpace.cpp
  GoalRegion.cpp
  MotionValidator.cpp
  Planner.cpp
  StateSampler.cpp
  StateValidityChecker.cpp
)

add_library("${PROJECT_NAME}_planner_ompl" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_planner_ompl" SYSTEM
  PUBLIC
    ${DART_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_planner_ompl"
  PUBLIC
    "${PROJECT_NAME}_common"
    "${PROJECT_NAME}_constraint"
    "${PROJECT_NAME}_distance"
    "${PROJECT_NAME}_trajectory"
    "${PROJECT_NAME}_statespace"
    "${PROJECT_NAME}_planner"
    ${DART_LIBRARIES}
    ${OMPL_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_planner_ompl"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} planner_ompl)
add_component_targets(${PROJECT_NAME} planner_ompl "${PROJECT_NAME}_planner_ompl")
add_component_dependencies(${PROJECT_NAME} planner_ompl
  constraint
  distance
  planner
  statespace
  trajectory
)

format_add_sources(${sources})
