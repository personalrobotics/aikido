#include "OMPLTestHelpers.hpp"
#include <gtest/gtest.h>
#include <aikido/ompl/dart.hpp>
#include <aikido/ompl/OMPLMotionValidator.hpp>
#include <aikido/ompl/AIKIDOGeometricStateSpace.hpp>
#include <aikido/ompl/AIKIDOStateValidityChecker.hpp>
#include <boost/make_shared.hpp>

TEST(OMPLMotionValidator, SimpleValidation)
{
  auto robot = createTranslationalRobot();
  auto stateSpace =
      std::make_shared<aikido::statespace::MetaSkeletonStateSpace>(robot);

  auto si = aikido::ompl::createSpaceInformation(stateSpace, make_rng());

  auto mockCollisionConstraint =
      std::make_shared<MockTranslationalRobotConstraint>(
          stateSpace, Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));
  std::vector<aikido::constraint::TestableConstraintPtr> constraints;
  constraints.emplace_back(mockCollisionConstraint);
  ::ompl::base::StateValidityCheckerPtr vchecker =
      boost::make_shared<aikido::ompl::AIKIDOStateValidityChecker>(si,
                                                                   constraints);
  si->setStateValidityChecker(vchecker);

  auto validator = std::make_shared<aikido::ompl::OMPLMotionValidator>(si, 0.1);

  auto state1 = si->allocState()
                    ->as<aikido::ompl::AIKIDOGeometricStateSpace::StateType>();
  setTranslationalState(Eigen::Vector3d(-5, -5, 0), stateSpace, state1->mState);
  auto state2 = si->allocState()
                    ->as<aikido::ompl::AIKIDOGeometricStateSpace::StateType>();

  setTranslationalState(Eigen::Vector3d(5, 5, 0), stateSpace, state2->mState);

  EXPECT_FALSE(validator->checkMotion(state1, state2));

  setTranslationalState(Eigen::Vector3d(-5, 5, 0), stateSpace, state2->mState);
  EXPECT_TRUE(validator->checkMotion(state1, state2));

  si->freeState(state1);
  si->freeState(state2);
}

TEST(OMPLMotionValidator, Resolution)
{
  auto robot = createTranslationalRobot();
  auto stateSpace =
      std::make_shared<aikido::statespace::MetaSkeletonStateSpace>(robot);

  auto si = aikido::ompl::createSpaceInformation(stateSpace, make_rng());

  auto mockCollisionConstraint =
      std::make_shared<MockTranslationalRobotConstraint>(
          stateSpace, Eigen::Vector3d(-0.1, -0.1, -0.1), Eigen::Vector3d(0.1, 0.1, 0.1));
  std::vector<aikido::constraint::TestableConstraintPtr> constraints;
  constraints.emplace_back(mockCollisionConstraint);
  ::ompl::base::StateValidityCheckerPtr vchecker =
      boost::make_shared<aikido::ompl::AIKIDOStateValidityChecker>(si,
                                                                   constraints);
  si->setStateValidityChecker(vchecker);


  auto state1 = si->allocState()
                    ->as<aikido::ompl::AIKIDOGeometricStateSpace::StateType>();
  setTranslationalState(Eigen::Vector3d(-0.2, -0.2, 0), stateSpace, state1->mState);
  auto state2 = si->allocState()
                    ->as<aikido::ompl::AIKIDOGeometricStateSpace::StateType>();

  setTranslationalState(Eigen::Vector3d(5, 5, 0), stateSpace, state2->mState);

  auto validator1 = std::make_shared<aikido::ompl::OMPLMotionValidator>(si, 0.5);
  EXPECT_TRUE(validator1->checkMotion(state1, state2));

  auto validator2 = std::make_shared<aikido::ompl::OMPLMotionValidator>(si, 0.15);
  EXPECT_FALSE(validator2->checkMotion(state1, state2));

  si->freeState(state1);
  si->freeState(state2);
}
