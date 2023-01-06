# Find hardware_interface
#
# This sets the following variables:
# hardware_interface_FOUND
# hardware_interface_INCLUDE_DIRS
# hardware_interface_LIBRARIES
# hardware_interface_VERSION

# Note: This find module is necessary because the config file imports "gtest",
# "tests", and "run_tests" that conflict with the targets defined by Aikido.

find_package(PkgConfig QUIET REQUIRED)

# Check to see if pkgconfig is installed.
set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH TRUE)
pkg_check_modules(PC_hardware_interface hardware_interface QUIET)

# Include directories
find_path(hardware_interface_INCLUDE_DIRS
  HINTS ${PC_hardware_interface_INCLUDE_DIRS}
)

# Libraries
find_library(hardware_interface_LIBRARIES
  hardware_interface
  HINTS ${PC_hardware_interface_LIBRARY_DIRS}
)

# Version
set(hardware_interface_VERSION ${PC_hardware_interface_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(hardware_interface
    FOUND_VAR hardware_interface_FOUND
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS hardware_interface_INCLUDE_DIRS hardware_interface_LIBRARIES
    VERSION_VAR   hardware_interface_VERSION
)
