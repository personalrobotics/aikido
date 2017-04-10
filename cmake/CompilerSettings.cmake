#==============================================================================
# Compiler settings
#
# Aikido currently supports four compilers: GCC, Clang (and AppleClang), and
# MSVC. CompilerSettings optionally requires a custon CMake option
# TREAT_WARNINGS_AS_ERRORS.

if(CMAKE_COMPILER_IS_GNUCXX)

  # Aikido requires GCC 4.8 or greater to fully support C++11.
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8)
    message(FATAL_ERROR "The installed g++ version is
        ${CMAKE_CXX_COMPILER_VERSION}. ${PROJECT_NAME} requires g++ 4.8 or
        greater."
    )
  endif()

  set(AIKIDO_CXX_STANDARD_FLAGS -std=c++11)

  add_compile_options(-Wall -Wextra)
  add_compile_options(-Wpedantic)
  if(TREAT_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()

elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

  # Aikido requires Clang 3.3 or greater to fully support C++11.
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.3)
    message(FATAL_ERROR "The installed Clang version is
        ${CMAKE_CXX_COMPILER_VERSION}. ${PROJECT_NAME} requires clang 3.3 or
        greater."
    )
  endif()

  set(AIKIDO_CXX_STANDARD_FLAGS -std=c++11)

  add_compile_options(-Wall)
  if(TREAT_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()

elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang")

  # Aikido requires AppleClang 6.1 or greater to fully support C++11.
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 6.1)
    message(FATAL_ERROR "The installed AppleClang version is
        ${CMAKE_CXX_COMPILER_VERSION}. ${PROJECT_NAME} requires AppleClang 6.1
        or greater."
    )
  endif()

  set(AIKIDO_CXX_STANDARD_FLAGS -std=c++11)

  add_compile_options(-Wall)
  if(TREAT_WARNINGS_AS_ERRORS)
    add_compile_options(-Werror)
  endif()

elseif(MSVC)

  # Aikido requires Visual Studio 2015 or greater to fully support C++11.
  if(MSVC_VERSION VERSION_LESS 1900)
     message(FATAL_ERROR "${PROJECT_NAME} requires VS 2015 or greater.")
   endif()

   # We don't need to specify an additional definition to enable build with
   # C++11 (e.g., -std=c++11) since Visual Studio enables it by default when
   # available.

  add_compile_options(/Wall)
  if(TREAT_WARNINGS_AS_ERRORS)
    add_compile_options(/WX)
  endif()

else()

  message(SEND_ERROR "Compiler[${CMAKE_CXX_COMPILER_ID}] is not supported.")

endif()
