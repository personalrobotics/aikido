# GTest setup.
add_subdirectory(gtest)
include_directories("${gtest_SOURCE_DIR}/include")

add_subdirectory("constraint")
add_subdirectory("distance")
add_subdirectory("ompl")
add_subdirectory("path")
add_subdirectory("planner")
add_subdirectory("statespace")
add_subdirectory("util")
