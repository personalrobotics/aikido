#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/ConfigurationToConfigurations.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include "../constraint/MockConstraints.hpp"

using aikido::trajectory::Interpolated;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::ConfigurationToConfigurations;

//==============================================================================
class ConfigurationToConfigurationTest : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using DistanceMetric = aikido::distance::DistanceMetric;
  using MetaSkeletonStateSpace
      = aikido::statespace::dart::MetaSkeletonStateSpace;
  using SO2 = aikido::statespace::SO2;
  using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  ConfigurationToConfigurationTest()
    : mSkel{dart::dynamics::Skeleton::create("skel")}
    , mJointAndBodyNode{mSkel->createJointAndBodyNodePair<dart::dynamics::
                                                            RevoluteJoint>()}
    , mStateSpace{std::make_shared<MetaSkeletonStateSpace>(mSkel.get())}
    , mStartState(ScopedState(mStateSpace->createState()))
    , mGoalState(ScopedState(mStateSpace->createState()))
    , mPassingConstraint{std::make_shared<PassingConstraint>(mStateSpace)}
  {
    mGoalStates.reserve(3);
  }

  // DART setup
  SkeletonPtr mSkel;
  std::pair<JointPtr, BodyNodePtr> mJointAndBodyNode;

  // Arguments for planner
  std::shared_ptr<MetaSkeletonStateSpace> mStateSpace;
  ScopedState mStartState;
  ScopedState mGoalState;
  ConfigurationToConfigurations::GoalStates mGoalStates;
  std::shared_ptr<PassingConstraint> mPassingConstraint;
};

//==============================================================================
TEST_F(ConfigurationToConfigurationTest, Basic)
{
  auto problem = ConfigurationToConfiguration(
      mStateSpace, mStartState, mGoalState, mPassingConstraint);

  EXPECT_EQ(problem.getType(), ConfigurationToConfiguration::getStaticType());

  EXPECT_EQ(problem.getStateSpace(), mStateSpace);
  EXPECT_EQ(problem.getConstraint(), mPassingConstraint);

  EXPECT_EQ(problem.getStartState(), mStartState);
  EXPECT_EQ(problem.getGoalState(), mGoalState);
}
