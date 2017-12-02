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
  dart/MetaSkeletonStateSpaceSaver.cpp
  dart/SE2Joint.cpp
  dart/SE3Joint.cpp
  dart/SO2Joint.cpp
  dart/SO3Joint.cpp
  dart/RnJoint.cpp
  dart/WeldJoint.cpp
)

add_library("${PROJECT_NAME}_statespace" SHARED ${sources})
target_include_directories("${PROJECT_NAME}_statespace"
  PUBLIC ${DART_INCLUDE_DIRS}
)
target_link_libraries("${PROJECT_NAME}_statespace"
  PUBLIC ${DART_LIBRARIES}
)
target_compile_options("${PROJECT_NAME}_statespace"
  PUBLIC ${AIKIDO_CXX_STANDARD_FLAGS}
)

add_component(${PROJECT_NAME} statespace)
add_component_targets(${PROJECT_NAME} statespace "${PROJECT_NAME}_statespace")

format_add_sources(${sources})
