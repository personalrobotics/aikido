aikido_add_library("${PROJECT_NAME}_path"
#  BlendTrajectory.cpp
#  ShiftTrajectory.cpp
#  CartesianTwistTrajectory.cpp
  PiecewiseLinearTrajectory.cpp
)
target_link_libraries("${PROJECT_NAME}_path"
  ${DART_LIBRARIES}
)
