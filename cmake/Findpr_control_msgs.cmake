# Find pr_control_msgs
#
# This sets the following variables:
# pr_control_msgs_FOUND
# pr_control_msgs_INCLUDE_DIRS
# pr_control_msgs_VERSION

# Note: This find module is necessary because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.

find_package(PkgConfig QUIET REQUIRED)

# Check to see if pkgconfig is installed.
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
pkg_check_modules(PC_pr_control_msgs pr_control_msgs QUIET)

# Include directories
find_path(pr_control_msgs_INCLUDE_DIRS
  NAMES pr_control_msgs/TriggerResult.h
  HINTS ${PC_pr_control_msgs_INCLUDE_DIRS}
)

# Version
set(pr_control_msgs_VERSION ${PC_pr_control_msgs_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(pr_control_msgs
    FOUND_VAR pr_control_msgs_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS pr_control_msgs_INCLUDE_DIRS
    VERSION_VAR   pr_control_msgs_VERSION
)
