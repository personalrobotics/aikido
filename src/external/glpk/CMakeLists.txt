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
add_library("${PROJECT_NAME}_external_glpk" STATIC IMPORTED GLOBAL)
add_dependencies("${PROJECT_NAME}_external_glpk" glpk_EXTERNAL)
set_target_properties("${PROJECT_NAME}_external_glpk" PROPERTIES
  IMPORTED_LOCATION "${INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}glpk${CMAKE_STATIC_LIBRARY_SUFFIX}"
  INTERFACE_POSITION_INDEPENDENT_CODE TRUE
)

# Ideally would add this directory to the INTERFACE_INCLUDE_DIRS property of
# the IMPORTed target. Unfortunately, that is not possible for two reasons:
# 1. it must occur BEFORE any system include paths to override the any system
#    version of GLPK, which is likely too recent for MinTOS
# 2. the directory is created at build time and CMake requires that all
#    directories in INTERFACE_INCLUDE_DIRECTORIES exist at compile time
set(glpk_EXTERNAL_INCLUDE_DIRS "${INSTALL_DIR}/include" CACHE INTERNAL "")
