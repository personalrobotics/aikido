# Copyright (c) 2014 Andrew Kelley
# This file is MIT licensed.
# See http://opensource.org/licenses/MIT

# TinyXML2_FOUND
# TinyXML2_INCLUDE_DIRS
# TinyXML2_LIBRARIES

find_path(TinyXML2_INCLUDE_DIRS NAMES tinyxml2.h)
find_library(TinyXML2_LIBRARIES NAMES tinyxml2)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TinyXML2 DEFAULT_MSG TinyXML2_LIBRARIES TinyXML2_INCLUDE_DIRS)

mark_as_advanced(TinyXML2_INCLUDE_DIRS TinyXML2_LIBRARIES)
