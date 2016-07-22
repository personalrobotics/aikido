# CMake 2.8.12 or above is required for CMakeParseArguments.
# CMake 2.8.11 or above is required for target_include_directories.
cmake_minimum_required(VERSION 2.8.12)
project(aikido)

set(_CATKIN_CONFIG_ALL_INCLUDED_ true)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake") 

#==============================================================================
# Configuration Options
#

set(INCLUDE_INSTALL_DIR "include")
set(LIBRARY_INSTALL_DIR "lib")
set(CONFIG_INSTALL_DIR "${LIBRARY_INSTALL_DIR}/${PROJECT_NAME}/cmake")

option(COVERALLS "Enable coveralls support" OFF)
option(COVERALLS_UPLOAD "Upload coveralls JSON output" ON)
option(DOWNLOAD_TAGFILES "Download Doxygen tagfiles for dependencies" OFF)

#==============================================================================
# Coveralls Setup
#
define_property(GLOBAL PROPERTY COVERALLS_SOURCES
  BRIEF_DOCS "Source files that Coveralls evaluates for test coverage."
  FULL_DOCS "Source files that Coveralls evaluates for test coverage."
)

function(coveralls_add_sources)
  foreach(source ${ARGN})
    get_filename_component(source_abs
      "${CMAKE_CURRENT_LIST_DIR}/${source}" ABSOLUTE)
    if(EXISTS "${source_abs}")
      set_property(GLOBAL APPEND PROPERTY COVERALLS_SOURCES "${source_abs}")
    else()
      message(FATAL_ERROR
        "Source file '${source}' does not exist at absolute path"
        " '${source_abs}'. This should never happen. Did you recently delete"
        " this file or modify 'CMAKE_CURRENT_LIST_DIR'")
    endif()
  endforeach()
endfunction()

if(COVERALLS)
  include(Coveralls)
  coveralls_turn_on_coverage()
endif()

#==============================================================================
# Helper functions.
#
function(evaluate_condition output_variable)
  if(${ARGN})
    set("${output_variable}" TRUE PARENT_SCOPE)
  else()
    set("${output_variable}" FALSE PARENT_SCOPE)
  endif()
endfunction()

function(aikido_space_delimit output_variable input_list)
  set(accumulator)
  foreach(input_element ${input_list})
    set(accumulator "${accumulator} ${input_element}")
  endforeach()

  set("${output_variable}" "${accumulator}" PARENT_SCOPE)
endfunction()

macro(aikido_check_package variable component dependency)
  if(${${variable}_FOUND})
    message(STATUS "Looking for ${dependency} - version ${${variable}_VERSION}"
                   " found")
  else()
    message(STATUS "Looking for ${dependency} - NOT found, to use"
                   " ${component}, please install ${dependency}")
    return()
  endif()
endmacro()

#==============================================================================
# Register an Aikido test.
#
set_property(GLOBAL PROPERTY AIKIDO_TESTS)

function(aikido_add_test target_name)
  add_executable("${target_name}" ${ARGN})
  add_test("${target_name}" "${target_name}")

  target_link_libraries("${target_name}" gtest gtest_main)

  set_property(GLOBAL APPEND PROPERTY AIKIDO_TESTS "${target_name}")
endfunction()

#==============================================================================
# Required Dependencies
#
include(Components)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)

find_package(Boost COMPONENTS filesystem)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

# Find DART in CONFIG mode to avoid finding built-in "FindDart.cmake" file,
# which shipped with CMake, on case-insensitive file systems. This is
# particularly a problem on OS X, which is generally case-insensitive.
# TODO: utils-urdf should only be required for aikido::perception
# TODO: utils is only included to workaround a bug in DART
find_package(DART REQUIRED COMPONENTS optimizer-nlopt utils-urdf CONFIG)
include_directories(SYSTEM ${DART_INCLUDE_DIRS})

#==============================================================================
# Libraries and unit tests.
#
# TODO: Add this to each target's DEFINITIONS property.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("include")

# Create "test" and "run_tests" targets to mimic a Catkin package. "tests"
# builds tests and "run_tests" (a synonym for "test") runs them. This MUST
# occur before we find_package() any Catkin packages because they attempt to
# re-define these targets.
add_custom_target(tests)
add_custom_target(run_tests COMMAND "${CMAKE_CTEST_COMMAND}")

set(CATKIN_ENABLE_TESTING false)

add_subdirectory("src")

enable_testing()
add_subdirectory("tests" EXCLUDE_FROM_ALL)

get_property(all_tests GLOBAL PROPERTY AIKIDO_TESTS)
add_dependencies(tests ${all_tests})

#==============================================================================
# Doxygen.
#
find_package(Doxygen QUIET)

if(${DOXYGEN_FOUND})
  message(STATUS "Looking for Doxygen - version ${DOXYGEN_VERSION} found")

  set(DOXYGEN_INPUT_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include")
  set(DOXYGEN_OUTPUT_ROOT "${CMAKE_CURRENT_BINARY_DIR}/doxygen")
  set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_OUTPUT_ROOT}/aikido.tag")
  set(DOXYGEN_HTML_INDEX "${DOXYGEN_OUTPUT_ROOT}/index.html")
  set(DOXYGEN_WORKING_DIR "${CMAKE_CURRENT_BINARY_DIR}/doxygen_working")
  set(DOXYGEN_EXCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/src/external")

  # Strip include_directories() being used by CMake from #include paths.
  # TODO: Add per-target INCLUDE_DIRECTORIES properties to this list.
  get_property(DOXYGEN_STRIP_FROM_INC_PATH_LIST
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    PROPERTY INCLUDE_DIRECTORIES)
  aikido_space_delimit(DOXYGEN_STRIP_FROM_INC_PATH
    "${DOXYGEN_STRIP_FROM_INC_PATH_LIST}")

  # Optionally download Doxygen tagfiles for dependencies.
  set(DOXYGEN_TAGFILES "")
  if(DOWNLOAD_TAGFILES)
    set(tagfile_url "http://dartsim.github.io/dart/dart.tag")
    message(STATUS "Downloading DART Doxygen tagfile from: ${tagfile_url}")
    file(DOWNLOAD "${tagfile_url}" "${DOXYGEN_WORKING_DIR}/dart.tag")
    set(DOXYGEN_TAGFILES
      "${DOXYGEN_TAGFILES} dart.tag=http://dartsim.github.io/dart/")

    is_component(needs_ompl planner_ompl)
    if(needs_ompl)
      set(tagfile_url "http://ompl.kavrakilab.org/core/ompl.tag")
      message(STATUS "Downloading OMPL Doxygen tagfile from: ${tagfile_url}")
      file(DOWNLOAD "${tagfile_url}" "${DOXYGEN_WORKING_DIR}/ompl.tag")
      set(DOXYGEN_TAGFILES
        "${DOXYGEN_TAGFILES} ompl.tag=http://ompl.kavrakilab.org/")
    endif()
  endif()

  # Generate a Doxyfile. This uses the variables:
  #
  # - DOXYGEN_EXCLUDE
  # - DOXYGEN_EXTRA_INPUTS
  # - DOXYGEN_GENERATE_TAGFILE
  # - DOXYGEN_INPUT_ROOT
  # - DOXYGEN_OUTPUT_ROOT
  # - DOXYGEN_STRIP_FROM_INC_PATH
  # - DOXYGEN_TAGFILES
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in"
    "${DOXYGEN_WORKING_DIR}/Doxyfile" @ONLY)

  # Create the "docs" target that runs Doxygen.
  add_custom_command(OUTPUT "${DOXYGEN_HTML_INDEX}"
    COMMAND "${DOXYGEN_EXECUTABLE}"
    DEPENDS "${DOXYGEN_WORKING_DIR}/Doxyfile"
    WORKING_DIRECTORY "${DOXYGEN_WORKING_DIR}")

  add_custom_target(docs
    DEPENDS "${DOXYGEN_HTML_INDEX}"
    COMMENT "Generating documentation.")

  # Create the "view_docs" target that opens the documentation.
  if(APPLE)
    set(open_command "open")
  else()
    set(open_command "xdg-open")
  endif()

  add_custom_target(view_docs "${open_command}" "${DOXYGEN_HTML_INDEX}"
    DEPENDS "${DOXYGEN_HTML_INDEX}"
    COMMENT "Opening documentation in a web browser.")
else()
  message(STATUS "Looking for Doxygen - NOT found, to generate API"
                 " documentation, please install Doxygen")
endif()

#==============================================================================
# Coveralls.
#
if(COVERALLS)
  get_property(sources GLOBAL PROPERTY COVERALLS_SOURCES)

  list(LENGTH sources sources_length)
  message(STATUS "Running Coveralls on ${sources_length} source files.")

  coveralls_setup("${sources}" ${COVERALLS_UPLOAD}
    "${PROJECT_SOURCE_DIR}/cmake")
endif()

#==============================================================================
# Installation.
#
install(DIRECTORY "include/"
  DESTINATION "${INCLUDE_INSTALL_DIR}")

# Install the package.xml file (to satisfy REP-136).
install(FILES "package.xml"
  DESTINATION "share/${PROJECT_NAME}")

# Generate and install CMake configuration files for each component <C>:
# - <C>Component.cmake, which defines:
#   - aikido_<C>_DEPENDENCIES: list of component dependencies
#   - aikido_<C>_LIBRARIES: list of library targets in this component
# - <C>Targets.cmake, which creates IMPORTED targets
install_component_exports()

# Generate and install a Config.cmake file. This file includes the
# <C>Component.cmake and <C>Targets.cmake created above. It also uses the
# following variables:
#
# - PACKAGE_INCLUDE_INSTALL_DIR
# - PACKAGE_INCLUDE_DIRS
get_property(PACKAGE_INCLUDE_DIRS GLOBAL
  PROPERTY "${PROJECT_NAME}_INCLUDE_DIRS")
configure_package_config_file("cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
  PATH_VARS INCLUDE_INSTALL_DIR)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  DESTINATION "${CONFIG_INSTALL_DIR}")
