#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/CollisionFree.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/planner/optimization/ConfigurationSpacePathLengthFunction.hpp>
#include <aikido/planner/optimization/MetaSkeletonSplineCoefficientsAndDurationsVariable.hpp>
#include <aikido/planner/optimization/SplineCoefficientsVariable.hpp>
#include <aikido/planner/optimization/TrajectoryOptimizer.hpp>
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
    , mDistanceMetric{distance::createDistanceMetricFor<MetaSkeletonStateSpace>(
          mStateSpace)}
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
  distance::DistanceMetricPtr mDistanceMetric;
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

  planner::optimization::SplineCoefficientsAndDurationsVariable variable(
      spline);
  EXPECT_TRUE(variable.getDimension() == 1 * 3 * 3 + 3);
}

//==============================================================================
TEST_F(OptimizationBasedMotionPlanner, PlanToConfiguration)
{
  using planner::optimization::TrajectoryOptimizer;
  using planner::optimization::
      MetaSkeletonSplineCoefficientsAndDurationsVariable;

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

  MetaSkeletonSplineCoefficientsAndDurationsVariable variable(
      spline, mSkeleton);
  //  planner::optimization::SplineCoefficientsVariables variable(spline);
  EXPECT_TRUE(variable.getDimension() == 1 * 3 * 3 + 3);
  //  EXPECT_TRUE(variable.getDimension() == 1 * 3 * 3);

  TrajectoryOptimizer planner(variable);

  auto objective = std::
      make_shared<planner::optimization::ConfigurationSpacePathLengthFunction>(
          mDistanceMetric);

  planner.setObjective(objective);

  Eigen::VectorXd x0(variable.getDimension());
  variable.setCoefficientValueAsJointMidPointsOfLimitsTo(x0);
  variable.setCoefficientValueTo(x0, 0.0);
  variable.setCoefficientValueAsCurrentJointPositionsTo(x0);
  variable.setDurationValueTo(x0, 0.15);
  planner.setInitialGuess(x0);

  Eigen::VectorXd lb(variable.getDimension());
  variable.setCoefficientValueAsJointPositionLowerLimitsTo(lb);
  variable.setCoefficientValueTo(lb, -1.0);
  variable.setDurationValueTo(lb, 0.15);
  planner.setLowerBounds(lb);

  Eigen::VectorXd ub(variable.getDimension());
  variable.setCoefficientValueAsJointPositionUpperLimitsTo(ub);
  variable.setCoefficientValueTo(ub, 1.0);
  variable.setDurationValueTo(ub, 0.2);
  planner.setUpperBounds(ub);

  EXPECT_TRUE(mStateSpace->getDimension() == 1);

  auto outcome = planner.createOutCome();
  planner.solve(outcome.get());

  std::cout << "x0: " << x0.transpose() << std::endl;
  std::cout << "lb: " << lb.transpose() << std::endl;
  std::cout << "ub: " << ub.transpose() << std::endl;
  std::cout << "solved: " << outcome->mMinimized << std::endl;
  std::cout << "f0: " << outcome->mInitialFunctionEvaluation << std::endl;
  std::cout << "f1: " << outcome->mMinimalFunctionEvaluation << std::endl;
}
