#include <gtest/gtest.h>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
// #include <Eigen/Core>

class SnapPlannerTest: ::testing::Test
{
public:
  using RealVectorStateSpace = aikido::statespace::RealVectorStateSpace;
  using ScopedState = RealVectorStateSpace::ScopedState;

  SnapPlannerTest()
  {
  }
};
