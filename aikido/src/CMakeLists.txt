add_subdirectory("external/hauser_parabolic_smoother")

add_subdirectory("util")
add_subdirectory("statespace")
add_subdirectory("distance")   # Dependencies: statespace
add_subdirectory("trajectory") # Dependencies: statespace
add_subdirectory("constraint") # Dependencies: statespace, util
add_subdirectory("planner")    # Dependencies: external, statespace, trajectory
