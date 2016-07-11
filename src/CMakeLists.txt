add_subdirectory("external/hauser_parabolic_smoother")

add_subdirectory("perception")
add_subdirectory("rviz")
add_subdirectory("statespace")
add_subdirectory("util")
add_subdirectory("distance")   # Dependencies: statespace
add_subdirectory("trajectory") # Dependencies: statespace
add_subdirectory("constraint") # Dependencies: statespace, util
add_subdirectory("planner")    # Dependencies: external, statespace, trajectory
add_subdirectory("control")    # Dependencies: statespace, trajectory, util
add_subdirectory("python")     # Dependencies: everything
