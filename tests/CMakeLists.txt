# GTest setup
add_library(gtest STATIC "${CMAKE_CURRENT_SOURCE_DIR}/gtest/src/gtest-all.cc")
target_include_directories(gtest SYSTEM BEFORE
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/gtest/include"
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/gtest"
)
if(NOT WIN32)
  target_link_libraries(gtest PUBLIC pthread)
endif()
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main PUBLIC gtest)

# Define aikido_add_test for registering Aikido tests.
set_property(GLOBAL PROPERTY AIKIDO_TESTS)
function(aikido_add_test target_name)
  add_executable("${target_name}" ${ARGN})
  add_test("${target_name}" "${target_name}")

  target_link_libraries("${target_name}" gtest gtest_main)

  set_property(GLOBAL APPEND PROPERTY AIKIDO_TESTS "${target_name}")
  format_add_sources(${ARGN})
endfunction()

# Add helper scripts to the include path.
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory("common")
add_subdirectory("io")
add_subdirectory("constraint")
add_subdirectory("control")
add_subdirectory("distance")
add_subdirectory("planner")
add_subdirectory("statespace")
add_subdirectory("trajectory")

format_add_sources(eigen_tests.hpp)
