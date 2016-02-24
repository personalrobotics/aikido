################################
# GTest setup
################################
add_subdirectory(gtest)
include_directories(${gtest_SOURCE_DIR}/include)

# For each source file in the list provided in the first argument, generate
# a unit-test that links against the specified libraries.
function(run_tests sources libraries)
  list(REMOVE_AT ARGV 0)
  foreach(source IN LISTS sources)
    get_filename_component(testname ${source} NAME_WE)
    add_executable("${testname}" "${source}")
    target_link_libraries("${testname}" ${ARGV})
    add_test("${testname}" "${testname}")
  endforeach(source)
endfunction()

################################
# Unit test declarations
################################
set(AIKIDO_TESTS
  test_TSR.cpp
  # test_SplineTrajectory.cpp
)
run_tests("${AIKIDO_TESTS}" aikido gtest gtest_main)
