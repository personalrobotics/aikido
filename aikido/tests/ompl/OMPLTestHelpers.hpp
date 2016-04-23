#ifndef OMPL_TEST_HELPERS_H_
#define OMPL_TEST_HELPERS_H_

#include <aikido/constraint/TestableConstraint.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/util/RNG.hpp>
#include <dart/dart.h>
#include <ompl/base/State.h>

using dart::common::make_unique;
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
    const aikido::statespace::MetaSkeletonStateSpacePtr &_stateSpace,
    ::ompl::base::State *_state)
{
  auto st = _state->as<aikido::ompl::AIKIDOGeometricStateSpace::StateType>();
  auto subState =
      _stateSpace->getSubStateHandle<aikido::statespace::RealVectorStateSpace>(
          st->mState, 0);
  subState.setValue(_value);
}

Eigen::Vector3d getTranslationalState(
    const aikido::statespace::MetaSkeletonStateSpacePtr &_stateSpace,
    ::ompl::base::State *_state)
{
  auto st = _state->as<aikido::ompl::AIKIDOGeometricStateSpace::StateType>();
  auto subState =
      _stateSpace->getSubStateHandle<aikido::statespace::RealVectorStateSpace>(
          st->mState, 0);
  return subState.getValue();

}
class MockTranslationalRobotConstraint
    : public aikido::constraint::TestableConstraint
{
public:
  // Construct the mock constraint
  MockTranslationalRobotConstraint(
      aikido::statespace::MetaSkeletonStateSpacePtr _stateSpace,
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
    auto subState =
        mStateSpace
            ->getSubStateHandle<aikido::statespace::RealVectorStateSpace>(
                _state, 0);
    auto val = subState.getValue();

    for (size_t i = 0; i < 3; ++i) {
      if (val[i] < mLowerBounds[i] || val[i] > mUpperBounds[i]) {
        return true;
      }
    }

    return false;
  }

  // Documentatoin inherited
  std::shared_ptr<aikido::statespace::StateSpace> getStateSpace() const override
  {
    return mStateSpace;
  }

private:
  aikido::statespace::MetaSkeletonStateSpacePtr mStateSpace;
  Eigen::Vector3d mUpperBounds;
  Eigen::Vector3d mLowerBounds;
};

#endif
