#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/CollisionFree.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/planner/optimization/Planner.hpp>
#include <aikido/planner/optimization/SplineCoefficientsAndDurationsVariables.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include "../../constraint/MockConstraints.hpp"

using namespace aikido;

using std::shared_ptr;
using std::make_shared;

class OptimizationBasedMotionPlanner : public ::testing::Test
{
public:
  using FCLCollisionDetector = dart::collision::FCLCollisionDetector;
  using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;
  using CollisionFree = aikido::constraint::CollisionFree;
  using DistanceMetric = aikido::distance::DistanceMetric;
  using MetaSkeletonStateSpace
      = aikido::statespace::dart::MetaSkeletonStateSpace;
  using SO2 = aikido::statespace::SO2;
  using GeodesicInterpolator = aikido::statespace::GeodesicInterpolator;
  using ScopedState = MetaSkeletonStateSpace::ScopedState;

  using BodyNodePtr = dart::dynamics::BodyNodePtr;
  using JointPtr = dart::dynamics::JointPtr;
  using SkeletonPtr = dart::dynamics::SkeletonPtr;

  OptimizationBasedMotionPlanner()
    : mSkeleton{dart::dynamics::Skeleton::create("skel")}
    , mJointAndBodyNode{mSkeleton
                            ->createJointAndBodyNodePair<dart::dynamics::
                                                             RevoluteJoint>()}
    , mStateSpace{make_shared<MetaSkeletonStateSpace>(mSkeleton)}
    , mStartState{make_shared<ScopedState>(mStateSpace->createState())}
    , mGoalState{make_shared<ScopedState>(mStateSpace->createState())}
    , mPassingConstraint{make_shared<PassingConstraint>(mStateSpace)}
    , mFailingConstraint{make_shared<FailingConstraint>(mStateSpace)}
    , mInterpolator(make_shared<GeodesicInterpolator>(mStateSpace))
  {
    // Do nothing
  }

  // DART setup
  SkeletonPtr mSkeleton;
  std::pair<JointPtr, BodyNodePtr> mJointAndBodyNode;

  // Arguments for planner
  shared_ptr<MetaSkeletonStateSpace> mStateSpace;
  shared_ptr<ScopedState> mStartState;
  shared_ptr<ScopedState> mGoalState;
  shared_ptr<PassingConstraint> mPassingConstraint;
  shared_ptr<FailingConstraint> mFailingConstraint;
  shared_ptr<GeodesicInterpolator> mInterpolator;
  planner::PlanningResult mPlanningResult;
};

//==============================================================================
TEST_F(OptimizationBasedMotionPlanner, Variables)
{
  using CoefficientType = Eigen::Matrix<double, 1, 3>;

  CoefficientType coefficients1 = CoefficientType::Zero();
  CoefficientType coefficients2 = CoefficientType::Zero();
  CoefficientType coefficients3 = CoefficientType::Zero();

  trajectory::Spline spline(mStateSpace);
  spline.addSegment(coefficients1, 1.0, mStartState->getState());
  spline.addSegment(coefficients2, 2.0);
  spline.addSegment(coefficients3, 3.0);
  EXPECT_TRUE(spline.getNumSegments() == 3);
  EXPECT_TRUE(spline.getDuration() == 6.0);

  planner::optimization::SplineCoefficientsAndDurationsVariables variables(spline);
  EXPECT_TRUE(variables.getDimension() == 1*3*3 + 3);
}

//==============================================================================
TEST_F(OptimizationBasedMotionPlanner, PlanToConfiguration)
{
  using CoefficientType = Eigen::Matrix<double, 1, 3>;

  CoefficientType coefficients1 = CoefficientType::Zero();
  CoefficientType coefficients2 = CoefficientType::Zero();
  CoefficientType coefficients3 = CoefficientType::Zero();

  trajectory::Spline spline(mStateSpace);
  spline.addSegment(coefficients1, 1.0, mStartState->getState());
  spline.addSegment(coefficients2, 2.0);
  spline.addSegment(coefficients3, 3.0);
  EXPECT_TRUE(spline.getNumSegments() == 3);
  EXPECT_TRUE(spline.getDuration() == 6.0);

  planner::optimization::SplineCoefficientsAndDurationsVariables variables(spline);
  EXPECT_TRUE(variables.getDimension() == 1*3*3 + 3);

  planner::optimization::OptimizationBasedMotionPlanning planner(variables);

  planner.setVariables(variables);

  EXPECT_TRUE(mStateSpace->getDimension() == 1);

  planner.plan();
}
