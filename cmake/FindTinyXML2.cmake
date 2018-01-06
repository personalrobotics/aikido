# Copyright (c) 2011-2017, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

# Find TinyXML2
#
# This sets the following variables:
# TinyXML2_FOUND
# TinyXML2_INCLUDE_DIRS
# TinyXML2_LIBRARIES
# TinyXML2_VERSION

find_package(PkgConfig QUIET)

# Check if the pkgconfig file is installed
pkg_check_modules(PC_TINYXML2 tinyxml2 QUIET)

# Include directories
find_path(TinyXML2_INCLUDE_DIRS
    NAMES tinyxml2.h
    HINTS ${PC_TinyXML2_INCLUDEDIR}
    PATHS "${CMAKE_INSTALL_PREFIX}/include")

# Libraries
if(MSVC)
  set(TinyXML2_LIBRARIES optimized tinyxml2 debug tinyxml2d)
else()
  find_library(TinyXML2_LIBRARIES
      NAMES tinyxml2
      HINTS ${PC_TinyXML2_LIBDIR})
endif()

# Version
set(TinyXML2_VERSION ${PC_TINYXML2_VERSION})

# Set (NAME)_FOUND if all the variables and the version are satisfied.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML2
    FAIL_MESSAGE  DEFAULT_MSG
    REQUIRED_VARS TinyXML2_INCLUDE_DIRS TinyXML2_LIBRARIES
    VERSION_VAR   TinyXML2_VERSION)
