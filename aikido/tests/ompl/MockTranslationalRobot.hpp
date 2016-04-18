#ifndef MOCK_TRANSLATIONAL_ROBOT_H_
#define MOCK_TRANSLATIONAL_ROBOT_H_

#include <dart/dart.h>
#include <aikido/constraint/TestableConstraint.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

dart::dynamics::SkeletonPtr createTranslationalRobot()
{
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
