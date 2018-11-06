#ifndef AIKIDO_TESTS_OMPL_OMPLTESTHELPERS_HPP_
#define AIKIDO_TESTS_OMPL_OMPLTESTHELPERS_HPP_

#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <ompl/base/State.h>
#include <aikido/common/RNG.hpp>
#include <aikido/constraint.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>

using dart::common::make_unique;
using aikido::constraint::DefaultTestableOutcome;
using aikido::constraint::TestableOutcome;
using aikido::statespace::CartesianProduct;
using aikido::statespace::R3;
using aikido::planner::ompl::GeometricStateSpace;
using aikido::common::RNGWrapper;
using aikido::common::RNG;
using DefaultRNG = RNGWrapper<std::default_random_engine>;

static std::unique_ptr<DefaultRNG> make_rng()
{
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

dart::dynamics::SkeletonPtr createTranslationalRobot()
{
  // Create robot
  auto robot = dart::dynamics::Skeleton::create("robot");
  robot->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint>();

  // Set bounds on the skeleton
  robot->setPositionLowerLimit(0, -5);
  robot->setPositionUpperLimit(0, 5);
  robot->setPositionLowerLimit(1, -5);
  robot->setPositionUpperLimit(1, 5);
  robot->setPositionLowerLimit(2, 0);
  robot->setPositionUpperLimit(2, 0);

  return robot;
}

void setTranslationalState(
    const Eigen::Vector3d& _value,
    const aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr& _stateSpace,
    ::ompl::base::State* _state)
{
  auto st = _state->as<aikido::planner::ompl::GeometricStateSpace::StateType>();
  auto cst = static_cast<CartesianProduct::State*>(st->mState);
  auto subState = _stateSpace->getSubStateHandle<R3>(cst, 0);
  subState.setValue(_value);
}

Eigen::Vector3d getTranslationalState(
    const aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr& _stateSpace,
    ::ompl::base::State* _state)
{
  auto st = _state->as<GeometricStateSpace::StateType>();
  auto cst = static_cast<CartesianProduct::State*>(st->mState);
  auto subState = _stateSpace->getSubStateHandle<R3>(cst, 0);

  return subState.getValue();
}

class MockConstrainedSampleGenerator
    : public aikido::constraint::SampleGenerator
{

public:
  MockConstrainedSampleGenerator(
      aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr _stateSpace,
      std::unique_ptr<aikido::constraint::SampleGenerator> _generator,
      double _value)
    : mStateSpace(std::move(_stateSpace))
    , mSampleGenerator(std::move(_generator))
    , mValue(_value)
  {
  }

  aikido::statespace::ConstStateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  /// Returns one sample from this constraint; returns true if succeeded.
  bool sample(aikido::statespace::StateSpace::State* _state) override
  {
    mSampleGenerator->sample(_state);
    auto outstate = static_cast<CartesianProduct::State*>(_state);
    auto outSubState = mStateSpace->getSubStateHandle<R3>(outstate, 0);
    auto val = outSubState.getValue();
    Eigen::Vector3d newval(mValue, val(1), val(2));
    outSubState.setValue(newval);
    return true;
  }

  /// Gets an upper bound on the number of samples remaining or NO_LIMIT.
  int getNumSamples() const override
  {
    return mSampleGenerator->getNumSamples();
  }

  /// Returns whether getNumSamples() > 0.
  bool canSample() const override
  {
    return mSampleGenerator->canSample();
  }

private:
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr mStateSpace;
  std::unique_ptr<aikido::constraint::SampleGenerator> mSampleGenerator;
  double mValue;
};

class MockProjectionConstraint : public aikido::constraint::Projectable,
                                 public aikido::constraint::Testable,
                                 public aikido::constraint::Sampleable
{
public:
  // Construct a constraint that project to x=_val
  MockProjectionConstraint(
      aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr _stateSpace,
      aikido::constraint::SampleablePtr _sampleable,
      double _val)
    : mStateSpace(std::move(_stateSpace))
    , mSampleable(std::move(_sampleable))
    , mValue(_val)
  {
  }

  // Documentation inherited
  aikido::statespace::ConstStateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  // Documentation inherited
  bool project(
      const aikido::statespace::StateSpace::State* _s,
      aikido::statespace::StateSpace::State* _out) const override
  {
    auto instate = static_cast<const CartesianProduct::State*>(_s);
    auto outstate = static_cast<CartesianProduct::State*>(_out);
    auto inSubState = mStateSpace->getSubStateHandle<R3>(instate, 0);
    auto outSubState = mStateSpace->getSubStateHandle<R3>(outstate, 0);
    auto val = inSubState.getValue();
    Eigen::Vector3d newval(mValue, val(1), val(2));
    outSubState.setValue(newval);
    return true;
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* _s,
      TestableOutcome* outcome = nullptr) const override
  {
    auto defaultOutcomeObject
        = aikido::constraint::dynamic_cast_or_throw<DefaultTestableOutcome>(
            outcome);

    auto state = static_cast<const CartesianProduct::State*>(_s);
    auto val = mStateSpace->getSubStateHandle<R3>(state, 0).getValue();

    bool isSatisfiedResult = std::fabs(val[0] - mValue) < 1e-6;
    if (defaultOutcomeObject)
      defaultOutcomeObject->setSatisfiedFlag(isSatisfiedResult);
    return isSatisfiedResult;
  }

  std::unique_ptr<TestableOutcome> createOutcome() const override
  {
    return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
  }

  std::unique_ptr<aikido::constraint::SampleGenerator> createSampleGenerator()
      const override
  {
    return make_unique<MockConstrainedSampleGenerator>(
        mStateSpace, mSampleable->createSampleGenerator(), mValue);
  }

private:
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr mStateSpace;
  aikido::constraint::SampleablePtr mSampleable;
  double mValue;
};

class MockTranslationalRobotConstraint : public aikido::constraint::Testable
{
public:
  // Construct the mock constraint
  MockTranslationalRobotConstraint(
      aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr _stateSpace,
      const Eigen::Vector3d& _lowerBounds,
      const Eigen::Vector3d& _upperBounds)
    : mStateSpace(std::move(_stateSpace))
    , mLowerBounds(_lowerBounds)
    , mUpperBounds(_upperBounds)
  {
  }

  // Documentation inherited
  bool isSatisfied(
      const aikido::statespace::StateSpace::State* _state,
      TestableOutcome* outcome = nullptr) const override
  {
    auto defaultOutcomeObject
        = aikido::constraint::dynamic_cast_or_throw<DefaultTestableOutcome>(
            outcome);

    auto cst = static_cast<const CartesianProduct::State*>(_state);
    auto subState = mStateSpace->getSubStateHandle<R3>(cst, 0);
    auto val = subState.getValue();

    for (std::size_t i = 0; i < 3; ++i)
    {
      if (val[i] < mLowerBounds[i] || val[i] > mUpperBounds[i])
      {
        if (defaultOutcomeObject)
          defaultOutcomeObject->setSatisfiedFlag(true);
        return true;
      }
    }

    if (defaultOutcomeObject)
      defaultOutcomeObject->setSatisfiedFlag(false);
    return false;
  }

  std::unique_ptr<TestableOutcome> createOutcome() const override
  {
    return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
  }

  // Documentation inherited
  aikido::statespace::ConstStateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

private:
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr mStateSpace;
  Eigen::Vector3d mLowerBounds;
  Eigen::Vector3d mUpperBounds;
};

class EmptySampleGenerator : public aikido::constraint::SampleGenerator
{
public:
  explicit EmptySampleGenerator(aikido::statespace::ConstStateSpacePtr sspace)
    : mStateSpace(sspace)
  {
  }

  aikido::statespace::ConstStateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool sample(aikido::statespace::StateSpace::State* /*_state*/) override
  {
    return true;
  }
  int getNumSamples() const override
  {
    return 0;
  }
  bool canSample() const override
  {
    return false;
  }

private:
  aikido::statespace::ConstStateSpacePtr mStateSpace;
};

class FailedSampleGenerator : public aikido::constraint::SampleGenerator
{
public:
  explicit FailedSampleGenerator(aikido::statespace::ConstStateSpacePtr sspace)
    : mStateSpace(sspace)
  {
  }

  aikido::statespace::ConstStateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool sample(aikido::statespace::StateSpace::State* /*_state*/) override
  {
    return false;
  }
  int getNumSamples() const override
  {
    return 1000;
  }
  bool canSample() const override
  {
    return true;
  }

private:
  aikido::statespace::ConstStateSpacePtr mStateSpace;
};

class PlannerTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;

    // Create robot
    robot = createTranslationalRobot();

    stateSpace = std::make_shared<const StateSpace>(robot.get());
    interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(
        stateSpace);

    // Collision constraint
    collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace,
        Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

    // Distance metric
    dmetric = aikido::distance::createDistanceMetric(stateSpace);

    // Sampler
    sampler
        = aikido::constraint::createSampleableBounds(stateSpace, make_rng());

    // Projectable constraint
    boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);

    // Joint limits
    boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);

    maxDistanceBetweenValidityCheck = 0.2;
  }

  dart::dynamics::SkeletonPtr robot;
  aikido::statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace;
  aikido::statespace::InterpolatorPtr interpolator;
  aikido::distance::DistanceMetricPtr dmetric;
  aikido::constraint::SampleablePtr sampler;
  aikido::constraint::ProjectablePtr boundsProjection;
  aikido::constraint::TestablePtr boundsConstraint;
  aikido::constraint::TestablePtr collConstraint;
  double maxDistanceBetweenValidityChecks;
};

class SimplifierTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;

    // Create robot
    robot = createTranslationalRobot();

    stateSpace = std::make_shared<StateSpace>(robot.get());
    interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(
        stateSpace);

    // Collision constraint
    collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace,
        Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

    // Distance metric
    dmetric = aikido::distance::createDistanceMetric(stateSpace);

    // Sampler
    sampler
        = aikido::constraint::createSampleableBounds(stateSpace, make_rng());

    // Projectable constraint
    boundsProjection = aikido::constraint::createProjectableBounds(stateSpace);

    // Joint limits
    boundsConstraint = aikido::constraint::createTestableBounds(stateSpace);
  }

  dart::dynamics::SkeletonPtr robot;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace;
  aikido::statespace::InterpolatorPtr interpolator;
  aikido::distance::DistanceMetricPtr dmetric;
  aikido::constraint::SampleablePtr sampler;
  aikido::constraint::ProjectablePtr boundsProjection;
  aikido::constraint::TestablePtr boundsConstraint;
  aikido::constraint::TestablePtr collConstraint;
};

#endif
