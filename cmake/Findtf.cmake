# Find tf
#
# This sets the following variables:
# tf_FOUND
# tf_INCLUDE_DIRS
# tf_LIBRARIES
# tf_VERSION

# Note: This find module is necessary because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.

find_package(PkgConfig QUIET REQUIRED)

# Check to see if pkgconfig is installed.
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
pkg_check_modules(PC_tf tf QUIET)

# Include directories
find_path(tf_INCLUDE_DIRS
  NAMES tf/tf.h
  HINTS ${PC_tf_INCLUDE_DIRS}
)

# Libraries
find_library(tf_LIBRARIES
  tf
  HINTS ${PC_tf_LIBRARY_DIRS}
)

# Version
set(tf_VERSION ${PC_tf_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(tf
    FOUND_VAR tf_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS tf_INCLUDE_DIRS
    VERSION_VAR   tf_VERSION
)
