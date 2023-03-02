# Find controller_manager_msgs
#
# This sets the following variables:
# controller_manager_msgs_FOUND
# controller_manager_msgs_INCLUDE_DIRS
# controller_manager_msgs_LIBRARIES
# controller_manager_msgs_VERSION

# Note: This find module is necessary because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.

find_package(PkgConfig QUIET REQUIRED)

# Check to see if pkgconfig is installed.
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
pkg_check_modules(PC_controller_manager_msgs controller_manager_msgs QUIET)

# Include directories
find_path(controller_manager_msgs_INCLUDE_DIRS
  HINTS ${PC_controller_manager_msgs_INCLUDE_DIRS}
)

# Libraries
find_library(controller_manager_msgs_LIBRARIES
  controller_manager_msgs
  HINTS ${PC_controller_manager_msgs_LIBRARY_DIRS}
)

# Version
set(controller_manager_msgs_VERSION ${PC_controller_manager_msgs_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(controller_manager_msgs
    FOUND_VAR controller_manager_msgs_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS controller_manager_msgs_INCLUDE_DIRS controller_manager_msgs_LIBRARIES
    VERSION_VAR   controller_manager_msgs_VERSION
)
