include(CMakeParseArguments)

set(component_prefix "${PROJECT_NAME}_component_")
set(export_prefix "${component_prefix}")
string(TOUPPER "${component_prefix}" property_prefix)

set(prefix "${PROJECT_NAME}_")

define_property(GLOBAL PROPERTY "${prefix}INCLUDE_DIRS"
  BRIEF_DOCS "Global include directories used by all components."
  FULL_DOCS "Global include directories used by all components."
)
define_property(GLOBAL PROPERTY "${prefix}COMPONENTS"
  BRIEF_DOCS "List all known Aikido components."
  FULL_DOCS "List all known Aikido components."
)

#==============================================================================
function(set_component_property component property)
  cmake_parse_arguments(args "APPEND" "" "" ${ARGN})

  string(TOUPPER "${component}" component_upper)
  string(TOUPPER "${property}" property_upper)

  if(args_APPEND)
    set_property(GLOBAL APPEND PROPERTY
      "${prefix}COMPONENT_${component_upper}_${property_upper}"
      ${args_UNPARSED_ARGUMENTS}
    )
  else()
    set_property(GLOBAL PROPERTY
      "${prefix}COMPONENT_${component_upper}_${property_upper}"
      ${args_UNPARSED_ARGUMENTS}
    )
  endif()
endfunction()

#==============================================================================
function(get_component_property output component property)
  cmake_parse_arguments(args "DEFINED" "" "" ${ARGN})

  string(TOUPPER "${component}" component_upper)
  string(TOUPPER "${property}" property_upper)

  if(args_DEFINED)
    get_property(property_value GLOBAL PROPERTY
      "${prefix}COMPONENT_${component_upper}_${property_upper}" ${extra_args}
      DEFINED
    )
  else()
    get_property(property_value GLOBAL PROPERTY
      "${prefix}COMPONENT_${component_upper}_${property_upper}" ${extra_args}
    )
  endif()

  set("${output}" ${property_value} PARENT_SCOPE)
endfunction()

#==============================================================================
function(open_component component)
  cmake_parse_arguments(args "REQUIRED" "" "DEPENDENCIES" ${ARGN})
  set(dependencies ${args_DEPENDENCIES})
  set(is_required ${args_REQUIRED})

  # Check for a duplicate component.
  get_component_property(status_defined "${component}" STATUS DEFINED)
  if(${status_defined})
    message(FATAL_ERROR
      "Attempted to add duplicate component '${component}'.")
  endif()

  message(STATUS "+++ Component: ${component}")

  # Check if the dependencies were found.
  set(dependencies_found TRUE)

  foreach(dependency ${dependencies})
    get_component_property(dependency_status "${dependency}" "STATUS")

    if("${dependency_status}" STREQUAL "CLOSED")
      message(STATUS "Looking for component ${dependency} - found")
    else()
      message(STATUS "Looking for component ${dependency} - not found")
      set(dependencies_found FALSE)
    endif()
  endforeach()

  if(${is_required} AND NOT ${dependencies_found})
    message(FATAL_ERROR
      "Skipping component ${component} because one or more required"
      " components were not found.")
  endif()

  # Register the component with Aikido.
  set_component_property("${component}" STATUS "OPEN")
  set_component_property("${component}" FOUND ${dependencies_found})
  set_component_property("${component}" REQUIRED "${is_required}")
  set_component_property("${component}" DEPENDENCIES ${dependencies})
  set_component_property("${component}" INCLUDE_DIRS)
  set_component_property("${component}" LIBRARIES)
  set_property(GLOBAL APPEND PROPERTY "${prefix}COMPONENTS" "${component}")
endfunction()
#==============================================================================
function(close_component component)
  cmake_parse_arguments(args "FAILED" "" "" ${ARGN})

  get_component_property(status "${component}" STATUS)
  if(NOT "${status}" STREQUAL "OPEN")
    message(FATAL_ERROR
      "Failed closing component '${component}' because it is not 'OPEN'.")
  endif()

  set_component_property("${component}" STATUS "CLOSED")

  if(args_FAILED)
    set_component_property("${component}" FOUND FALSE)
  endif()

  # Handle the REQUIRED option
  get_component_property(is_failed "${component}" FAILED)
  get_component_property(is_required "${component}" REQUIRED)

  if(is_failed AND is_required)
    message(FATAL_ERROR "Failed to create required component '${component}'.") 
  endif()
  
  # Add include_directories to this component's libraries.
  get_component_property(include_dirs "${component}" INCLUDE_DIRS)
  get_component_property(libraries "${component}" LIBRARIES)

  if(include_dirs)
    foreach(target ${libraries})
      target_include_directories("${target}" SYSTEM PUBLIC ${include_dirs})
    endforeach()
  endif()

  message(STATUS "--- Component: ${component}")
endfunction()

#==============================================================================
function(component_include_directories)
  cmake_parse_arguments(args "" "COMPONENT" "" ${ARGN})
  set(directories ${args_UNPARSED_ARGUMENTS})

  if(DEFINED args_COMPONENT)
    set(component ${args_COMPONENT})
    get_component_property(status "${component}" STATUS)

    if(NOT "${status}" STREQUAL "OPEN")
      message(FATAL_ERROR
        "Failed adding include_directories because component '${component}'"
        " is not open.")
    endif()

    set_component_property("${component}" INCLUDE_DIRS ${directories} APPEND)
  else()
    message(FATAL_ERROR "COMPONENT argument is required.")
  endif()

  # TODO: Switch to target_include_directories.
  include_directories(SYSTEM ${directories})
endfunction()

#==============================================================================
function(global_include_directories)
  set_property(GLOBAL APPEND PROPERTY "${prefix}INCLUDE_DIRS" ${ARGN})
  include_directories(SYSTEM ${ARGN})
endfunction()

#==============================================================================
function(add_component_library target)
  cmake_parse_arguments(args "EXTERNAL" "COMPONENT" "SOURCES" ${ARGN})
  set(relative_sources ${args_SOURCES})

  if(DEFINED args_COMPONENT)
    set(component ${args_COMPONENT})
    get_component_property(component_status "${component}" STATUS)

    if ("${component_status}" STREQUAL "OPEN")
      message(STATUS
        "Registered library '${target}' as part of component '${component}'."
      )
    else()
      message(FATAL_ERROR
        "Unable to add library '${target}' to component '${component}' because"
        " it is not open."
      )
    endif()

    set_component_property("${component}" LIBRARIES "${target}" APPEND)

    string(TOLOWER "${component}" component_lower)
    set(export "${PROJECT_NAME}_${component_lower}Targets")
  else()
    message(STATUS "Registered library '${target}'.")
    set(export "${PROJECT_NAME}Targets")
  endif()

  add_library("${target}" SHARED ${relative_sources})
  install(TARGETS "${target}"
    EXPORT "${export}"
    LIBRARY DESTINATION "${LIBRARY_INSTALL_DIR}"
  )

  # Compile a list of all source files to pass to coveralls. This function may
  # be called from a subdirctory, so we first convert to absolute paths.
  set(abs_sources)
  foreach(rel_source ${relative_sources})
    get_filename_component(abs_source
      "${CMAKE_CURRENT_SOURCE_DIR}/${rel_source}" ABSOLUTE
    )
    list(APPEND abs_sources "${abs_source}")
  endforeach()
endfunction()

#==============================================================================
function(install_component_exports)
  get_property(components GLOBAL PROPERTY "${prefix}COMPONENTS")

  set(output_prefix "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_INSTALL_DIR}")
  set(output_path "${output_prefix}/${PROJECT_NAME}Components.cmake")
  file(WRITE "${output_path}" "set(${prefix}COMPONENTS ${components})\n")

  foreach(component ${components})
    string(TOUPPER "${component}" component_upper)
    string(TOLOWER "${component}" component_lower)

    get_component_property(component_status "${component}" STATUS)
    if(NOT "${component_status}" STREQUAL "CLOSED")
      message(FATAL_ERROR "Component '${component}' was not closed.")
    endif()

    get_component_property(component_found "${component}" FOUND)
    set(variable "${prefix}${component_upper}_FOUND")
    file(APPEND "${output_path}" "set(${variable} ${component_found})\n")

    if(${component_found})
      get_component_property(include_dirs "${component}" INCLUDE_DIRS)
      get_component_property(libraries "${component}" LIBRARIES)
      get_component_property(dependencies "${component}" DEPENDENCIES)

      set(variable "${prefix}${component_upper}_INCLUDE_DIRS")
      file(APPEND "${output_path}" "set(${variable} ${include_dirs})\n")

      set(variable "${prefix}${component_upper}_LIBRARIES")
      file(APPEND "${output_path}" "set(${variable} ${libraries})\n")

      set(variable "${prefix}${component_upper}_DEPENDENCIES")
      file(APPEND "${output_path}" "set(${variable} ${dependencies})\n")

      if(libraries)
        install(EXPORT "${PROJECT_NAME}_${component_lower}Targets"
          FILE "${PROJECT_NAME}_${component_lower}Targets.cmake"
          DESTINATION "${CONFIG_INSTALL_DIR}"
        )
      endif()
    else()
      set(variable "${prefix}${component_upper}_INCLUDE_DIRS")
      file(APPEND "${output_path}" "set(${variable} ${variable}-NOTFOUND)\n")

      set(variable "${prefix}${component_upper}_LIBRARIES")
      file(APPEND "${output_path}" "set(${variable} ${variable}-NOTFOUND)\n")

      set(variable "${prefix}${component_upper}_DEPENDENCIES")
      file(APPEND "${output_path}" "set(${variable})\n")
    endif()
  endforeach()

  install(FILES "${output_path}"
    DESTINATION "${CONFIG_INSTALL_DIR}"
  )
endfunction()




#==============================================================================
function(add_component component)
  set(target "${component_prefix}${component}")
  add_custom_target("${target}")

  install(EXPORT "${export_prefix}${component}"
    FILE "${PROJECT_NAME}_${component}Targets.cmake"
    DESTINATION "${CONFIG_INSTALL_DIR}"
  )

  set_property(TARGET "${target}" PROPERTY "${property_prefix}COMPONENT" TRUE)
  set_property(TARGET "${target}" PROPERTY "${property_prefix}DEPENDENCIES")
  set_property(TARGET "${target}" PROPERTY "${property_prefix}TARGETS")
  set_property(GLOBAL APPEND PROPERTY "${prefix}COMPONENTS" "${component}")
endfunction()

#==============================================================================
function(is_component output_variable component)
  set(target "${component_prefix}${component}")

  if(NOT TARGET "${target}")
    message(FATAL_ERROR
      "'${component}' is not a component of ${PROJECT_NAME}.")
  endif()

  get_property(output TARGET "${target}"
    PROPERTY "${property_prefix}COMPONENT")
  set("${output_variable}" ${output} PARENT_SCOPE)
endfunction()

#==============================================================================
function(add_component_include_directories component)
  set(target "${component_prefix}${component}")
  set_property(TARGET "${target}" APPEND
    PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${ARGN})
endfunction()

#==============================================================================
function(add_component_dependencies component)
  set(dependency_components ${ARGN})

  is_component(is_valid "${component}")
  if(NOT ${is_valid})
    message(FATAL_ERROR
      "Target '${component}' is not a component of ${PROJECT_NAME}.")
  endif()

  set(target "${component_prefix}${component}")

  foreach(dependency_component ${dependency_components})
    is_component(is_valid "${dependency_component}")
    if(NOT ${is_valid})
      message(FATAL_ERROR
        "Target '${dependency_component}' is not a component of ${PROJECT_NAME}.")
    endif()

    set(dependency_target "${component_prefix}${dependency_component}")
    add_dependencies("${target}" "${dependency_target}")
  endforeach()

  set_property(TARGET "${target}" APPEND
    PROPERTY "${property_prefix}DEPENDENCIES" ${dependency_components})
endfunction()

#==============================================================================
function(add_component_targets component)
  set(dependency_targets ${ARGN})

  is_component(is_valid "${component}")
  if(NOT ${is_valid})
    message(FATAL_ERROR
      "Target '${component}' is not a component of ${PROJECT_NAME}.")
  endif()

  set(target "${component_prefix}${component}")
  add_dependencies("${target}" ${ARGN})

  foreach(dependency_target ${dependency_targets})
    install(TARGETS "${dependency_target}"
      EXPORT "${export_prefix}${component}"
      ARCHIVE DESTINATION "${LIBRARY_INSTALL_DIR}"
      LIBRARY DESTINATION "${LIBRARY_INSTALL_DIR}"
    )
  endforeach()

  set_property(TARGET "${target}" APPEND
    PROPERTY "${property_prefix}TARGETS" ${dependency_targets})
endfunction()

#==============================================================================
function(install_component_exports)
  get_property(components GLOBAL PROPERTY "${prefix}COMPONENTS")

  set(output_prefix "${CMAKE_CURRENT_BINARY_DIR}/${CONFIG_INSTALL_DIR}")

  foreach(component ${components})
    set(target "${component_prefix}${component}")

    # TODO: Replace this manual generation with a configure_file.
    set(output_path
      "${output_prefix}/${PROJECT_NAME}_${component}Component.cmake")
    file(WRITE "${output_path}" "")

    get_property(dependencies TARGET "${target}"
      PROPERTY "${property_prefix}DEPENDENCIES")
    file(APPEND "${output_path}"
      "set(\"${PROJECT_NAME}_${component}_DEPENDENCIES\" ${dependencies})\n")

    get_property(targets TARGET "${target}"
      PROPERTY "${property_prefix}TARGETS")
    file(APPEND "${output_path}"
      "set(\"${PROJECT_NAME}_${component}_TARGETS\" ${targets})\n")

    install(FILES "${output_path}"
      DESTINATION "${CONFIG_INSTALL_DIR}")
  endforeach()
endfunction()
