# GTest setup.
add_subdirectory(gtest)

# Add helper scripts to the include path.
include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory("constraint")
add_subdirectory("control")
add_subdirectory("distance")
add_subdirectory("planner")
add_subdirectory("statespace")
add_subdirectory("trajectory")
add_subdirectory("util")
