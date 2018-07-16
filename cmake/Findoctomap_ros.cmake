# Find octomap_ros
#
# This sets the following variables:
# octomap_ros_FOUND
# octomap_ros_INCLUDE_DIRS
# octomap_ros_VERSION

# Note: This find module is necessary because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.

find_package(PkgConfig QUIET REQUIRED)

# Check to see if pkgconfig is installed.
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
pkg_check_modules(PC_octomap_ros octomap_ros QUIET)

# Include directories
find_path(octomap_ros_INCLUDE_DIRS
  NAMES octomap_ros/conversions.h
  HINTS ${PC_octomap_ros_INCLUDE_DIRS}
)

# Version
set(octomap_ros_VERSION ${PC_octomap_ros_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(octomap_ros
    FOUND_VAR octomap_ros_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS octomap_ros_INCLUDE_DIRS
    VERSION_VAR   octomap_ros_VERSION
)
