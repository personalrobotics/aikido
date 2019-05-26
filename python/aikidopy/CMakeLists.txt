cmake_minimum_required(VERSION 2.8.12)

if(NOT AIKIDOPY_PYTHON_VERSION)
  set(AIKIDOPY_PYTHON_VERSION 3.4 CACHE STRING "Choose the target Python version (e.g., 3.4, 2.7)" FORCE)
endif()

find_package(PythonInterp ${AIKIDOPY_PYTHON_VERSION} REQUIRED)
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(NOT PythonInterp_FOUND)
  message(STATUS "BUILD_PYTHON is ON, but failed to find PythonInterp. "
    "Disabling aikidopy."
  )
  return()
endif()

find_package(PythonLibs ${AIKIDOPY_PYTHON_VERSION} QUIET)
if(NOT PythonLibs_FOUND)
  message(STATUS "BUILD_AIKIDOPY is ON, but failed to find PythonLibs. "
    "Disabling aikidopy."
  )
  return()
endif()

# Find pybind11
# Needs to set PYBIND11_PYTHON_VERSION before finding pybind11
set(PYBIND11_PYTHON_VERSION ${AIKIDOPY_PYTHON_VERSION})
find_package(pybind11 2.2.0 QUIET)
if(NOT pybind11_FOUND)
  message(STATUS "BUILD_PYTHON is ON, but failed to find pybind11 >= "
    "2.2.0. Disabling aikidopy."
  )
  return()
endif()

#================================================================================
# Dependencies
#

#================================================================================

file(GLOB_RECURSE aikidopy_headers "*.h" "*.hpp")
file(GLOB_RECURSE aikidopy_sources "*.cpp")

# Build a Python extension module:
# pybind11_add_module(<name> [MODULE | SHARED] [EXCLUDE_FROM_ALL]
#                     [NO_EXTRAS] [SYSTEM] [THIN_LTO] source1 [source2 ...])
#
pybind11_add_module(aikidopy
  MODULE
    ${aikidopy_headers}
    ${aikidopy_sources}
)

target_include_directories(aikidopy
  SYSTEM PUBLIC
    ${PYTHON_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(aikidopy
  PUBLIC
    aikido_common
    aikido_constraint
    aikido_control
    aikido_distance
    aikido_io
    aikido_planner
    aikido_robot
    aikido_statespace
    aikido_trajectory
    ${PYTHON_LIBRARIES}
)
if(TARGET aikido_control_ros)
  target_link_libraries(aikidopy PUBLIC aikido_control_ros)
endif()
if(TARGET aikido_perception)
  target_link_libraries(aikidopy PUBLIC aikido_perception)
endif()
if(TARGET aikido_rviz)
  target_link_libraries(aikidopy PUBLIC aikido_rviz)
endif()

set_target_properties(aikidopy
  PROPERTIES
    PREFIX ""
    SUFFIX ".so"  # python uses '.so' extension even on macOS
    DEBUG_POSTFIX ""
)

install(TARGETS aikidopy
  LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
)

list(REMOVE_ITEM aikidopy_headers
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.h
  ${CMAKE_CURRENT_LIST_DIR}/eigen_pybind.h
)
list(REMOVE_ITEM aikidopy_sources
  ${CMAKE_CURRENT_LIST_DIR}/eigen_geometry_pybind.cpp
)

clang_format_add_sources(${aikidopy_headers} ${aikidopy_sources})
