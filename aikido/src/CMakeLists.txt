add_subdirectory("external/hauser_parabolic_smoother")

add_subdirectory("util")
add_subdirectory("statespace")
add_subdirectory("path")       # Dependencies: statespace
add_subdirectory("constraint") # Dependencies: statespace, util
add_subdirectory("planner")    # Dependencies: external, statespace, path
add_subdirectory("ompl")       # Dependencies: constraint, statespace, path
add_subdirectory("distance")   # Dependencies: statespace
