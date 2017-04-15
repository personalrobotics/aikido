# Find actionlib
#
# This sets the following variables:
# actionlib_FOUND
# actionlib_INCLUDE_DIRS
# actionlib_LIBRARIES
# actionlib_VERSION

# Note: This find module is necessary because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.

find_package(PkgConfig QUIET REQUIRED)

# Check to see if pkgconfig is installed.
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
pkg_check_modules(PC_actionlib actionlib QUIET)

# Include directories
find_path(actionlib_INCLUDE_DIRS
  NAMES actionlib/action_definition.h
  HINTS ${PC_actionlib_LIBRARY_DIRS}
)

# Libraries
find_library(actionlib_LIBRARIES
  actionlib
  HINTS ${PC_actionlib_LIBRARY_DIRS}
)

# Version
set(actionlib_VERSION ${PC_actionlib_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(actionlib
    FOUND_VAR actionlib_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS actionlib_INCLUDE_DIRS actionlib_LIBRARIES
    VERSION_VAR   actionlib_VERSION
)
