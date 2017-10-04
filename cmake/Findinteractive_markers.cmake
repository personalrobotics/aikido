# Find interactive_markers
#
# This sets the following variables:
# interactive_markers_FOUND
# interactive_markers_INCLUDE_DIRS
# interactive_markers_LIBRARIES
# interactive_markers_VERSION

# Note: This find module is necessary because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.

find_package(PkgConfig QUIET REQUIRED)

# Check to see if pkgconfig is installed.
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
pkg_check_modules(PC_interactive_markers interactive_markers QUIET)

# Include directories
find_path(interactive_markers_INCLUDE_DIRS
  NAMES interactive_markers/interactive_marker_client.h
  HINTS ${PC_tf_INCLUDE_DIRS}
)

# Libraries
find_library(interactive_markers_LIBRARIES
  interactive_markers
  HINTS ${PC_interactive_markers_LIBRARY_DIRS}
)

# Version
set(interactive_markers_VERSION ${PC_interactive_markers_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(interactive_markers
    FOUND_VAR interactive_markers_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS interactive_markers_INCLUDE_DIRS
    VERSION_VAR   interactive_markers_VERSION
)
