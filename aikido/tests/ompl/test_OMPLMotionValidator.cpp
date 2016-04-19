#include "OMPLTestHelpers.hpp"
#include <gtest/gtest.h>
#include <aikido/ompl/dart.hpp>
#include <aikido/ompl/OMPLMotionValidator.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>
#include <aikido/ompl/AIKIDOStateValidityChecker.hpp>
#include <boost/make_shared.hpp>

/// This test creates a world with a translational robot
/// and a .2x.2x.2 block obstacle at the origin
class OMPLMotionValidatorTest : public ::testing::Test
{
public:
  virtual void SetUp()
  {
    auto robot = createTranslationalRobot();
    stateSpace =
        std::make_shared<aikido::statespace::MetaSkeletonStateSpace>(robot);

    si = aikido::ompl::createSpaceInformation(stateSpace, make_rng());

    auto mockCollisionConstraint =
        std::make_shared<MockTranslationalRobotConstraint>(
            stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1),
            Eigen::Vector3d(0.1, 0.1, 0.1));
    std::vector<aikido::constraint::TestableConstraintPtr> constraints;
    constraints.emplace_back(mockCollisionConstraint);
    ::ompl::base::StateValidityCheckerPtr vchecker =
        boost::make_shared<aikido::ompl::AIKIDOStateValidityChecker>(
            si, constraints);
    si->setStateValidityChecker(vchecker);
    validator = std::make_shared<aikido::ompl::OMPLMotionValidator>(si, 0.1);

    state1 = si->allocState();
    state2 = si->allocState();
  }

  virtual void TearDown()
  {
    si->freeState(state1);
    si->freeState(state2);
  }

  std::shared_ptr<aikido::ompl::OMPLMotionValidator> validator;
  ::ompl::base::State* state1;
  ::ompl::base::State* state2;
  ::ompl::base::SpaceInformationPtr si;
  aikido::statespace::MetaSkeletonStateSpacePtr stateSpace;
};

TEST_F(OMPLMotionValidatorTest, SuccessValidation)
{
  setTranslationalState(Eigen::Vector3d(-5, -5, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(-5, 5, 0), stateSpace, state2);
  EXPECT_TRUE(validator->checkMotion(state1, state2));
}

TEST_F(OMPLMotionValidatorTest, FailedValidation)
{
  setTranslationalState(Eigen::Vector3d(-5, -5, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(5, 5, 0), stateSpace, state2);
  EXPECT_FALSE(validator->checkMotion(state1, state2));
}

TEST_F(OMPLMotionValidatorTest, LastValidEmptyState)
{
  // Check that nothing throws if lastValid has no state
  setTranslationalState(Eigen::Vector3d(-5, -5, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(-5, 5, 0), stateSpace, state2);
  std::pair<::ompl::base::State*, double> lastValid;
  EXPECT_TRUE(validator->checkMotion(state1, state2, lastValid));
}

TEST_F(OMPLMotionValidatorTest, SuccessValidationLastValid)
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

TEST_F(OMPLMotionValidatorTest, FailedValidationLastValid)
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

TEST_F(OMPLMotionValidatorTest, BadValidationSmallResolution)
{
  setTranslationalState(Eigen::Vector3d(-0.2, -0.2, 0), stateSpace, state1);
  setTranslationalState(Eigen::Vector3d(5, 5, 0), stateSpace, state2);

  // Set stepsize large enough that validation shoudl succeed even with obstacle
  // in the way
  auto validator1 =
      std::make_shared<aikido::ompl::OMPLMotionValidator>(si, 0.5);
  EXPECT_TRUE(validator1->checkMotion(state1, state2));
}
