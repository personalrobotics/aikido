#include <boost/make_shared.hpp>
#include <gtest/gtest.h>

#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/planner/ompl/MotionValidator.hpp>
#include <aikido/planner/ompl/StateValidityChecker.hpp>
#include <aikido/planner/ompl/dart.hpp>

#include "../../constraint/MockConstraints.hpp"
#include "OMPLTestHelpers.hpp"

using aikido::planner::ompl::MotionValidator;
using aikido::planner::ompl::ompl_make_shared;
using aikido::statespace::dart::MetaSkeletonStateSpace;

/// This test creates a world with a translational robot
/// and a .2x.2x.2 block obstacle at the origin
class MotionValidatorTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    auto robot = createTranslationalRobot();
    stateSpace = std::make_shared<MetaSkeletonStateSpace>(robot.get());

    auto mockTestable = std::make_shared<PassingConstraint>(stateSpace);
    si = aikido::planner::ompl::createSpaceInformation(
        stateSpace, mockTestable, 0.1, make_rng());

    auto mockCollisionConstraint
        = std::make_shared<MockTranslationalRobotConstraint>(
            stateSpace,
            Eigen::Vector3d(-0.1, -0.1, -0.1),
            Eigen::Vector3d(0.1, 0.1, 0.1));
    ::ompl::base::StateValidityCheckerPtr vchecker
        = ompl_make_shared<aikido::planner::ompl::StateValidityChecker>(
            si, mockCollisionConstraint);
    si->setStateValidityChecker(vchecker);
    validator = std::make_shared<MotionValidator>(si, 0.1);

    state1 = si->allocState();
    state2 = si->allocState();
  }

  virtual void TearDown()
  {
    si->freeState(state1);
    si->freeState(state2);
  }

  std::shared_ptr<MotionValidator> validator;
  ::ompl::base::State* state1;
  ::ompl::base::State* state2;
  ::ompl::base::SpaceInformationPtr si;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr stateSpace;
};

TEST_F(MotionValidatorTest, ConstructorThrowsOnNullSpaceInformation)
{
  EXPECT_THROW(MotionValidator(nullptr, 0.1), std::invalid_argument);
}

TEST_F(MotionValidatorTest, ConstructorThrowsOnZeroDistance)
{
  EXPECT_THROW(MotionValidator(si, 0.0), std::invalid_argument);
}

TEST_F(MotionValidatorTest, ConstructorThrowsOnNegativeDistance)
{
  EXPECT_THROW(MotionValidator(si, -0.1), std::invalid_argument);
}

TEST_F(MotionValidatorTest, SuccessValidation)
{
  setTranslationalState(Eigen::Vector3d(-5, -5, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(-5, 5, 0), stateSpace, state2);
  EXPECT_TRUE(validator->checkMotion(state1, state2));
}

TEST_F(MotionValidatorTest, FailedValidation)
{
  setTranslationalState(Eigen::Vector3d(-5, -5, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(5, 5, 0), stateSpace, state2);
  EXPECT_FALSE(validator->checkMotion(state1, state2));
}

TEST_F(MotionValidatorTest, LastValidEmptyState)
{
  // Check that nothing throws if lastValid has no state
  setTranslationalState(Eigen::Vector3d(-5, -5, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(-5, 5, 0), stateSpace, state2);
  std::pair<::ompl::base::State*, double> lastValid;
  EXPECT_TRUE(validator->checkMotion(state1, state2, lastValid));
}

TEST_F(MotionValidatorTest, SuccessValidationLastValid)
{
  setTranslationalState(Eigen::Vector3d(-5, -5, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(-5, 5, 0), stateSpace, state2);

  std::pair<::ompl::base::State*, double> lastValid;
  lastValid.first = si->allocState();
  EXPECT_TRUE(validator->checkMotion(state1, state2, lastValid));
  EXPECT_DOUBLE_EQ(1.0, lastValid.second);
  EXPECT_TRUE(getTranslationalState(stateSpace, lastValid.first)
                  .isApprox(Eigen::Vector3d(-5, 5, 0.)));
}

TEST_F(MotionValidatorTest, FailedValidationLastValid)
{
  setTranslationalState(Eigen::Vector3d(0, -5, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(0, 5, 0), stateSpace, state2);

  std::pair<::ompl::base::State*, double> lastValid;
  lastValid.first = si->allocState();
  EXPECT_FALSE(validator->checkMotion(state1, state2, lastValid));
  EXPECT_DOUBLE_EQ((5 - 0.2) / 10, lastValid.second);
  EXPECT_TRUE(getTranslationalState(stateSpace, lastValid.first)
                  .isApprox(Eigen::Vector3d(0, -0.2, 0.)));
}

TEST_F(MotionValidatorTest, BadValidationSmallResolution)
{
  setTranslationalState(Eigen::Vector3d(-0.2, -0.2, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(5, 5, 0), stateSpace, state2);

  // Set stepsize large enough that validation should succeed even with obstacle
  // in the way
  auto validator1
      = std::make_shared<aikido::planner::ompl::MotionValidator>(si, 0.5);
  EXPECT_TRUE(validator1->checkMotion(state1, state2));
}
