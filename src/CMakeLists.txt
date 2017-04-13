# Trailing comments represent the dependencies of each subdirectory where the
# Aikido targets are marked as [target_name].

add_subdirectory("external/hauser_parabolic_smoother")

add_subdirectory("perception") # boost, dart, yaml-cpp, geometry_msgs, roscpp, std_msgs, visualization_msgs
add_subdirectory("rviz")       # boost, dart, roscpp, geometry_msgs, interactive_markers, std_msgs, visualization_msgs, libmicrohttpd
add_subdirectory("statespace") # dart
add_subdirectory("util")       # boost, dart, tinyxml2
add_subdirectory("distance")   # [statespace], dart
add_subdirectory("trajectory") # [statespace], [util]
add_subdirectory("constraint") # [statespace], [util]
add_subdirectory("planner")    # [external], [statespace], [trajectory], [constraint], [distance], [util], dart, ompl
add_subdirectory("control")    # [statespace], [trajectory]
#add_subdirectory("python")     # everything
