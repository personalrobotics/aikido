#ifndef AIKIDO_TESTS_OMPL_OMPLTESTHELPERS_HPP_
#define AIKIDO_TESTS_OMPL_OMPLTESTHELPERS_HPP_

#include <gtest/gtest.h>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/util/RNG.hpp>
#include <dart/dart.h>
#include <ompl/base/State.h>

using dart::common::make_unique;
using aikido::statespace::CartesianProduct;
using aikido::statespace::Rn;
using aikido::planner::ompl::GeometricStateSpace;
using aikido::util::RNGWrapper;
using aikido::util::RNG;
using DefaultRNG = RNGWrapper<std::default_random_engine>;

static std::unique_ptr<DefaultRNG> make_rng()
{
  return make_unique<RNGWrapper<std::default_random_engine>>(0);
}

dart::dynamics::SkeletonPtr createTranslationalRobot()
{
  // Create robot
  auto robot = dart::dynamics::Skeleton::create("robot");
  auto jn_bn =
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
    const Eigen::Vector3d &_value,
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &_stateSpace,
    ::ompl::base::State *_state)
{
  auto st = _state->as<aikido::planner::ompl::GeometricStateSpace::StateType>();
  auto cst = static_cast<CartesianProduct::State*>(st->mState);
  auto subState =
      _stateSpace->getSubStateHandle<aikido::statespace::Rn>(
          cst, 0);
  subState.setValue(_value);
}

Eigen::Vector3d getTranslationalState(
    const aikido::statespace::dart::MetaSkeletonStateSpacePtr &_stateSpace,
    ::ompl::base::State *_state)
{
  auto st = _state->as<GeometricStateSpace::StateType>();
  auto cst = static_cast<CartesianProduct::State*>(st->mState);
  auto subState =
      _stateSpace->getSubStateHandle<Rn>(
          cst, 0);
  return subState.getValue();

}

class MockProjectionConstraint : public aikido::constraint::Projectable,
                                 aikido::constraint::Testable {
public:
  // Construct a constraint that project to x=_val
  MockProjectionConstraint(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      double _val)
      : mStateSpace(std::move(_stateSpace))
      , mValue(_val)
  {
  }

  // Documentation inherited
  aikido::statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  // Documentation inherited
  bool project(const aikido::statespace::StateSpace::State *_s,
               aikido::statespace::StateSpace::State *_out) const override {
    auto instate =
        static_cast<const CartesianProduct::State *>(_s);
    auto outstate =
        static_cast<CartesianProduct::State *>(_out);
    auto inSubState =
        mStateSpace
            ->getSubStateHandle<Rn>(instate, 0);
    auto outSubState =
        mStateSpace
            ->getSubStateHandle<Rn>(outstate, 0);
    auto val = inSubState.getValue();
    Eigen::Vector3d newval(mValue, val(1), val(2));
    outSubState.setValue(newval);
  }

  bool
  isSatisfied(const aikido::statespace::StateSpace::State *_s) const override {
    auto state =
        static_cast<const CartesianProduct::State *>(_s);
    auto val = mStateSpace->getSubStateHandle<Rn>(state, 0).getValue();
    return std::fabs(val[0] - mValue) < 1e-6;
  }

private:
    aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
    double mValue;
};

class MockTranslationalRobotConstraint : public aikido::constraint::Testable
{
public:
  // Construct the mock constraint
  MockTranslationalRobotConstraint(
      aikido::statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      const Eigen::Vector3d &_lowerBounds, const Eigen::Vector3d &_upperBounds)
      : mStateSpace(std::move(_stateSpace))
      , mLowerBounds(_lowerBounds)
      , mUpperBounds(_upperBounds)
  {
  }

  // Documentation inherited
  bool isSatisfied(
      const aikido::statespace::StateSpace::State *_state) const override
  {
    auto cst =
        static_cast<const CartesianProduct::State *>(_state);
    auto subState =
        mStateSpace
            ->getSubStateHandle<Rn>(cst, 0);
    auto val = subState.getValue();

    for (size_t i = 0; i < 3; ++i) {
      if (val[i] < mLowerBounds[i] || val[i] > mUpperBounds[i]) {
        return true;
      }
    }

    return false;
  }

  // Documentatoin inherited
  std::shared_ptr<aikido::statespace::StateSpace>
  getStateSpace() const override {
    return mStateSpace;
  }

private:
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  Eigen::Vector3d mUpperBounds;
  Eigen::Vector3d mLowerBounds;
};


class EmptySampleGenerator : public aikido::constraint::SampleGenerator
{
public:
  EmptySampleGenerator(aikido::statespace::StateSpacePtr sspace)
      : mStateSpace(sspace)
  {
  }

  aikido::statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool sample(aikido::statespace::StateSpace::State* _state) override
  {
    return true;
  }
  int getNumSamples() const override { return 0; }
  bool canSample() const override { return false; }

private:
  aikido::statespace::StateSpacePtr mStateSpace;
};

class FailedSampleGenerator : public aikido::constraint::SampleGenerator {
public:
  FailedSampleGenerator(aikido::statespace::StateSpacePtr sspace)
      : mStateSpace(sspace)
  {
  }

  aikido::statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool sample(aikido::statespace::StateSpace::State* _state) override
  {
    return false;
  }
  int getNumSamples() const override { return 1000; }
  bool canSample() const override { return true; }

private:
  aikido::statespace::StateSpacePtr mStateSpace;
};

class PlannerTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    using StateSpace = aikido::statespace::dart::MetaSkeletonStateSpace;

    // Create robot
    robot = createTranslationalRobot();

    stateSpace = std::make_shared<StateSpace>(robot);
    interpolator = std::make_shared<aikido::statespace::GeodesicInterpolator>(stateSpace);

    // Collision constraint
    collConstraint = std::make_shared<MockTranslationalRobotConstraint>(
        stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1, 0.1, 0.1));

    // Distance metric
    dmetric = aikido::distance::createDistanceMetric(stateSpace);

    // Sampler
    sampler =
        aikido::constraint::createSampleableBounds(stateSpace, make_rng());

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
