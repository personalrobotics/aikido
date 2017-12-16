#include <tuple>
#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/constraint/CollisionFree.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/PlanningResult.hpp>
#include <aikido/planner/SnapPlanner.hpp>
#include <aikido/planner/optimization/CompositeFunction.hpp>
#include <aikido/planner/optimization/CompositeVariable.hpp>
#include <aikido/planner/optimization/ConfigurationSpacePathLengthFunction.hpp>
#include <aikido/planner/optimization/MetaSkeletonSplineCoefficientsAndDurationsVariable.hpp>
#include <aikido/planner/optimization/RnVariable.hpp>
#include <aikido/planner/optimization/SplineCoefficientsVariable.hpp>
#include <aikido/planner/optimization/TrajectoryOptimizer.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include "../../constraint/MockConstraints.hpp"

using namespace aikido;

using std::shared_ptr;
using std::make_shared;
using planner::optimization::Optimizer;
using planner::optimization::TrajectoryOptimizer;
using planner::optimization::Variable;
using planner::optimization::VariablePtr;
using planner::optimization::RVariable;
using planner::optimization::R2Variable;
using planner::optimization::R3Variable;
using planner::optimization::CompositeVariable;
using planner::optimization::Function;
using planner::optimization::CompositeFunction;
using planner::optimization::MetaSkeletonSplineCoefficientsAndDurationsVariable;

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
class NormFunction : public aikido::planner::optimization::Function
{
public:
  NormFunction(VariablePtr variable) : Function(std::move(variable))
  {

  }

  aikido::planner::optimization::UniqueFunctionPtr clone() const override
  {
    return dart::common::make_unique<NormFunction>(*this);
  }

  bool isCompatible(const Variable& /*variable*/) const override
  {
    return true;
  }

  double eval(const Eigen::VectorXd& x) override
  {
    return x.norm();
  }
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
TEST_F(OptimizationBasedMotionPlanner, CompositeVariable)
{
  auto compositeVar = std::make_shared<CompositeVariable>();
  EXPECT_EQ(compositeVar->getDimension(), 0u);

  auto r2Var = std::make_shared<R2Variable>();
  EXPECT_EQ(r2Var->getDimension(), 2u);

  auto r3Var = std::make_shared<R3Variable>();
  EXPECT_EQ(r3Var->getDimension(), 3u);

  compositeVar->addSubVariable(r2Var);
  EXPECT_EQ(compositeVar->getDimension(), 2u);
  EXPECT_EQ(compositeVar->getSubVariable(0u), r2Var);

  compositeVar->addSubVariable(r3Var);
  EXPECT_EQ(compositeVar->getDimension(), 5u);
  EXPECT_EQ(compositeVar->getSubVariable(1u), r3Var);

  compositeVar->addSubVariable(r3Var);
  EXPECT_EQ(compositeVar->getDimension(), 5u);
}

//==============================================================================
TEST_F(OptimizationBasedMotionPlanner, CompositeFunction)
{
  auto compositeVar = std::make_shared<CompositeVariable>();
  auto r2Var = std::make_shared<R2Variable>();
  auto r3Var = std::make_shared<R3Variable>();
  compositeVar->addSubVariable(r2Var);
  compositeVar->addSubVariable(r3Var);

  auto compositeFunction = std::make_shared<CompositeFunction>(compositeVar);
  auto normFunction2 = std::make_shared<NormFunction>(r2Var);
  compositeFunction->addSubFunction(normFunction2, r2Var);

  Eigen::VectorXd value = Eigen::VectorXd::Zero(5);
  value[0] = 1.0;
  value[2] = 2.0;

  EXPECT_FLOAT_EQ(compositeFunction->eval(value), 1.0);

  auto normFunction3 = std::make_shared<NormFunction>(r3Var);
  compositeFunction->addSubFunction(normFunction3, r3Var);

  EXPECT_FLOAT_EQ(compositeFunction->eval(value), 3.0);
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

  auto variable
      = std::make_shared<MetaSkeletonSplineCoefficientsAndDurationsVariable>(
          spline, mSkeleton);
  EXPECT_TRUE(variable->getDimension() == 1 * 3 * 3 + 3);

  TrajectoryOptimizer planner(*variable);

  auto objective = std::
      make_shared<planner::optimization::ConfigurationSpacePathLengthFunction>(
          variable, mDistanceMetric);

  planner.setObjective(objective);

  Eigen::VectorXd x0(variable->getDimension());
  variable->setCoefficientValueAsJointMidPointsOfLimitsTo(x0);
  variable->setCoefficientValueTo(x0, 0.0);
  variable->setCoefficientValueAsCurrentJointPositionsTo(x0);
  variable->setDurationValueTo(x0, 0.15);
  planner.setInitialGuess(x0);

  Eigen::VectorXd lb(variable->getDimension());
  variable->setCoefficientValueAsJointPositionLowerLimitsTo(lb);
  variable->setCoefficientValueTo(lb, -1.0);
  variable->setDurationValueTo(lb, 0.15);
  planner.setLowerBounds(lb);

  Eigen::VectorXd ub(variable->getDimension());
  variable->setCoefficientValueAsJointPositionUpperLimitsTo(ub);
  variable->setCoefficientValueTo(ub, 1.0);
  variable->setDurationValueTo(ub, 0.2);
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

  objective->setVariable(variable);
  Eigen::VectorXd value = x0;
  auto f2 = objective->eval(value);
  std::cout << "f2: " << f2 << std::endl;
}
