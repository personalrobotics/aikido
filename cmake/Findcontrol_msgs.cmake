# Find control_msgs
#
# This sets the following variables:
# control_msgs_FOUND
# control_msgs_INCLUDE_DIRS
# control_msgs_VERSION

# Note: This find module is necessary because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.

find_package(PkgConfig QUIET REQUIRED)

# Check to see if pkgconfig is installed.
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
pkg_check_modules(PC_control_msgs control_msgs QUIET)

# Include directories
find_path(control_msgs_INCLUDE_DIRS
  NAMES control_msgs/JointTolerance.h
  HINTS ${PC_control_msgs_INCLUDE_DIRS}
)

# Version
set(control_msgs_VERSION ${PC_control_msgs_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(control_msgs
    FOUND_VAR control_msgs_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS control_msgs_INCLUDE_DIRS
    VERSION_VAR   control_msgs_VERSION
)
