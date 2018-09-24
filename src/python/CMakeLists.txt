find_package(chimera QUIET)
aikido_check_package(chimera "Python bindings" "Chimera")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated -Wno-deprecated-declarations")


# Find pybind11
# Needs to set PYBIND11_PYTHON_VERSION before finding pybind11
set(PYBIND11_PYTHON_VERSION ${DARTPY_PYTHON_VERSION})
find_package(pybind11 2.2.0 REQUIRED)

find_package(PythonInterp ${AIKIDO_PYTHON_VERSION} QUIET)
aikido_check_package(PYTHONINTERP "Python bindings" "Python interpreter")
execute_process(COMMAND ${PYTHON_EXECUTABLE} -c
  "from distutils.sysconfig import get_python_lib;\
  print(get_python_lib(plat_specific=True, prefix=''))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PythonLibs ${AIKIDO_PYTHON_VERSION} QUIET)
aikido_check_package(PYTHONLIBS "Python bindings" "Python libraries")

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${DART_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)
include_directories(SYSTEM
  "${PROJECT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}"
)

get_property(AIKIDO_LIBRARIES GLOBAL PROPERTY AIKIDO_LIBRARIES)

# If chimera is available on this system, create a target to build bindings.
if(chimera_FOUND)
  add_chimera_binding(TARGET "${PROJECT_NAME}_python_CHIMERA"
    MODULE "${PROJECT_NAME}"
    CONFIGURATION "${CMAKE_CURRENT_SOURCE_DIR}/chimera.yml"
    NAMESPACES "aikido"
    SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/chimera.cpp"
    DESTINATION "${CMAKE_CURRENT_SOURCE_DIR}/src/python/generated"
    DEBUG
    EXCLUDE_FROM_ALL
  )
  target_link_libraries("${PROJECT_NAME}_python_CHIMERA"
    ${AIKIDO_LIBRARIES}
    ${DART_LIBRARIES}
    ${PYTHON_LIBRARIES}
  )
  add_custom_target(binding DEPENDS "${PROJECT_NAME}_python_CHIMERA_REBUILD")
else()
  add_custom_target(binding
    COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red "ERROR: Cannot generate bindings without \\'chimera\\' installed."
    COMMAND false
  )
endif()

# Create a target to build generated bindings.
set(SOURCES_TXT "${CMAKE_CURRENT_SOURCE_DIR}/src/python/generated/sources.txt")
if(EXISTS "${SOURCES_TXT}")
  file(STRINGS "${SOURCES_TXT}" SOURCES_GENERATED_RELATIVE NO_HEX_CONVERSION)
  
  set(SOURCES_GENERATED)
  foreach(relative_path ${SOURCES_GENERATED_RELATIVE})
    list(APPEND SOURCES_GENERATED "${CMAKE_CURRENT_SOURCE_DIR}/src/python/generated/${relative_path}")
  endforeach()

  add_library("${PROJECT_NAME}_python" MODULE
    ${SOURCES_GENERATED}
    # src/python/get_signature.hpp
  )

  target_include_directories("${PROJECT_NAME}_python"
    PUBLIC
      ${pybind11_INCLUDE_DIRS}
  )

  target_link_libraries("${PROJECT_NAME}_python"
    PUBLIC
      ${AIKIDO_LIBRARIES}
      ${DART_LIBRARIES}
      ${PYTHON_LIBRARIES}
  )
  
  set_target_properties("${PROJECT_NAME}_python" PROPERTIES
    PREFIX ""
    SUFFIX ".so"  # python uses '.so' extension even on macOS
  )

  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options("${PROJECT_NAME}_python"
      PRIVATE -fabi-version=6
    )
  endif()

  install(TARGETS "${PROJECT_NAME}_python"
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
  )
else()
  add_custom_target(${PROJECT_NAME}_python ALL
    COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red "ERROR: Generate bindings using \\'make binding\\' before attempting to build."
    COMMAND false
  )
endif()