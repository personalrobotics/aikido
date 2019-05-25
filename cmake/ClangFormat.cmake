function(clang_format_setup)
  cmake_parse_arguments(
        CF_ARG # prefix 
        "" # no boolean args
        "VERSION" # clang-format version
        "" # no multi-value args
        ${ARGN} 
    )
  if(NOT CF_ARG_VERSION)
    set(CF_NAME clang-format)
  else()
    set(CF_NAME clang-format-${CF_ARG_VERSION})
  endif()
  find_program(CLANG_FORMAT_EXECUTABLE NAMES ${CF_NAME})
  
  if(NOT CLANG_FORMAT_EXECUTABLE)
    message(STATUS "Looking for clang-format - NOT found, please install "
      "${CF_NAME} to enable automatic code formatting."
    )
    return()
  endif()

  message(STATUS "Found ${CF_NAME}.")
endfunction()

#===============================================================================
function(_property_add property_name)
  get_property(is_defined GLOBAL PROPERTY ${property_name} DEFINED)
  if(NOT is_defined)
    define_property(GLOBAL PROPERTY ${property_name}
      BRIEF_DOCS "${property_name}"
      FULL_DOCS "Global properties for ${property_name}"
    )
  endif()
  foreach(item ${ARGN})
    set_property(GLOBAL APPEND PROPERTY ${property_name} "${item}")
  endforeach()
endfunction()

#===============================================================================
function(clang_format_add_sources)
  foreach(source ${ARGN})
    if(IS_ABSOLUTE "${source}")
      set(source_abs "${source}")
    else()
      get_filename_component(source_abs
        "${CMAKE_CURRENT_LIST_DIR}/${source}" ABSOLUTE)
    endif()
    if(EXISTS "${source_abs}")
      _property_add(CLANG_FORMAT_FORMAT_FILES "${source_abs}")
    else()
      message(FATAL_ERROR
        "Source file '${source}' does not exist at absolute path"
        " '${source_abs}'. This should never happen. Did you recently delete"
        " this file or modify 'CMAKE_CURRENT_LIST_DIR'")
    endif()
  endforeach()
endfunction()

#===============================================================================
function(clang_format_add_targets)
  get_property(formatting_files GLOBAL PROPERTY CLANG_FORMAT_FORMAT_FILES)
  list(LENGTH formatting_files formatting_files_length)

  if(formatting_files)
    message(STATUS "Formatting on ${formatting_files_length} source files.")

    add_custom_target(format
      COMMAND ${CMAKE_COMMAND} -E echo "Formatting ${formatting_files_length} files..."
      COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${formatting_files}
      COMMAND ${CMAKE_COMMAND} -E echo "Done."
      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    add_custom_target(check-format
      COMMAND ${CMAKE_COMMAND} -E echo "Checking ${formatting_files_length} files..."
      COMMAND ${CMAKE_SOURCE_DIR}/tools/check_format.sh ${CLANG_FORMAT_EXECUTABLE} ${formatting_files}
      COMMAND ${CMAKE_COMMAND} -E echo "Done."
      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  else()
    add_custom_target(format
        COMMAND ${CMAKE_COMMAND} -E echo "No file to format code style."
    )
    add_custom_target(check-format
        COMMAND ${CMAKE_COMMAND} -E echo "No file to check code style."
    )
  endif()
endfunction()
