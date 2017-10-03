# Trailing comments represent the dependencies of each subdirectory where the
# Aikido targets are marked as [target_name].

add_subdirectory("external/hauser_parabolic_smoother")

add_subdirectory("common")     # boost, dart
add_subdirectory("io")         # [common], boost, dart, tinyxml2, yaml-cpp
add_subdirectory("rviz")       # boost, dart, roscpp, geometry_msgs, interactive_markers, std_msgs, visualization_msgs, libmicrohttpd
add_subdirectory("statespace") # dart
add_subdirectory("perception") # [io], boost, dart, yaml-cpp, geometry_msgs, roscpp, std_msgs, visualization_msgs
add_subdirectory("distance")   # [statespace], dart
add_subdirectory("trajectory") # [common], [statespace]
add_subdirectory("constraint") # [common], [statespace]
add_subdirectory("planner")    # [external], [common], [statespace], [trajectory], [constraint], [distance], dart, ompl
add_subdirectory("control")    # [statespace], [trajectory]
#add_subdirectory("python")     # everything
