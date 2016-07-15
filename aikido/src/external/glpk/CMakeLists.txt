# Build GLPK as as an external project.
ExternalProject_Add(glpk_EXTERNAL
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_COMMAND
    "${CMAKE_CURRENT_SOURCE_DIR}/configure"
    "--with-pic"
    "--enable-shared=no"
    "--enable-static=yes"
    "--prefix=<INSTALL_DIR>"
  BUILD_COMMAND "make"
  LOG_CONFIGURE 1
  LOG_BUILD 1
  LOG_INSTALL 1
)
ExternalProject_Get_Property(glpk_EXTERNAL INSTALL_DIR)

# Wrap GLPK in an IMPORTED target so CMake can use it as a dependency.
add_library(glpk STATIC IMPORTED GLOBAL)
set_property(TARGET glpk PROPERTY IMPORTED_LOCATION
  "${INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}glpk${CMAKE_STATIC_LIBRARY_SUFFIX}")
add_dependencies(glpk glpk_EXTERNAL)

set(glpk_INCLUDE_DIRS "${INSTALL_DIR}/include"
  CACHE PATH "GLPK include directories")
set(glpk_LIBRARIES "glpk"
  CACHE STRING "GLPK libraries")
