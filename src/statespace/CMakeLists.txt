set(sources
  StateSpace.cpp
  Rn.cpp
  CartesianProduct.cpp
  SE2.cpp
  SE3.cpp
  SO2.cpp
  SO3.cpp
  GeodesicInterpolator.cpp
  dart/JointStateSpace.cpp
  dart/JointStateSpaceHelpers.cpp
  dart/MetaSkeletonStateSpace.cpp
  dart/SE2Joint.cpp
  dart/SE3Joint.cpp
  dart/SO2Joint.cpp
  dart/SO3Joint.cpp
  dart/RnJoint.cpp
)

add_library("${PROJECT_NAME}_statespace" SHARED ${sources})
target_link_libraries("${PROJECT_NAME}_statespace"
  ${DART_LIBRARIES}
)

add_component(statespace)
add_component_targets(statespace "${PROJECT_NAME}_statespace")

coveralls_add_sources(${sources})
