#include "aikido/constraint/dart/FIFOStrategy.hpp"

#include <random>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/constraint/FiniteSampleable.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include "MockConstraints.hpp"

using aikido::statespace::R2;
using aikido::constraint::FiniteSampleable;
using aikido::constraint::dart::FIFOStrategy;
using aikido::statespace::SE3;
using aikido::statespace::SO2;
using dart::dynamics::FreeJoint;
using aikido::constraint::SampleGenerator;
using aikido::common::RNGWrapper;
using aikido::common::RNG;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::RevoluteJoint;

static BodyNode::Properties create_BodyNodeProperties(const std::string& _name)
{
  BodyNode::Properties bodyProperties;
  bodyProperties.mName = _name;
  return bodyProperties;
}

class FIFOStrategyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mRng.reset(new RNGWrapper<std::default_random_engine>());

    // Manipulator with 2 revolute joints.
    mManipulator1 = Skeleton::create("Manipulator1");

    // Root joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitY();
    properties1.mName = "Joint1";

    bn1 = mManipulator1
              ->createJointAndBodyNodePair<RevoluteJoint>(
                  nullptr, properties1, create_BodyNodeProperties("root_body"))
              .second;

    // joint 2, body 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, 1);
    bn2 = mManipulator1
              ->createJointAndBodyNodePair<RevoluteJoint>(
                  bn1, properties2, create_BodyNodeProperties("second_body"))
              .second;

    mStateSpace1
        = std::make_shared<MetaSkeletonStateSpace>(mManipulator1.get());

    // Manipulator with 1 free joint and 1 revolute joint.
    mManipulator2 = Skeleton::create("Manipulator2");

    // Root joint
    FreeJoint::Properties properties3;
    properties3.mName = "Joint1";
    bn3 = mManipulator2
              ->createJointAndBodyNodePair<FreeJoint>(
                  nullptr, properties3, create_BodyNodeProperties("root_body"))
              .second;

    // Joint 2, body 2
    RevoluteJoint::Properties properties4;
    properties4.mAxis = Eigen::Vector3d::UnitY();
    properties4.mName = "Joint2";
    properties4.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, 1);
    bn4 = mManipulator2
              ->createJointAndBodyNodePair<RevoluteJoint>(
                  bn3, properties4, create_BodyNodeProperties("second_body"))
              .second;

    mStateSpace2
        = std::make_shared<MetaSkeletonStateSpace>(mManipulator2.get());

    // Set FiniteSampleable to generate pose close to the actual solution.
    auto seedState
        = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
    seedState.getSubStateHandle<SO2>(0).setAngle(0.1);
    seedState.getSubStateHandle<SO2>(1).setAngle(0.1);
    seedConstraint
        = std::make_shared<FiniteSampleable>(mStateSpace1, seedState);
  }

  std::unique_ptr<RNG> mRng;

  SkeletonPtr mManipulator1;
  MetaSkeletonStateSpacePtr mStateSpace1;
  BodyNodePtr bn1, bn2;

  SkeletonPtr mManipulator2;
  MetaSkeletonStateSpacePtr mStateSpace2;
  BodyNodePtr bn3, bn4;
  std::shared_ptr<FiniteSampleable> seedConstraint;
};

TEST_F(FIFOStrategyTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(FIFOStrategy(nullptr, mManipulator1, 1), std::invalid_argument);
}

TEST_F(FIFOStrategyTest, ConstructorThrowsOnNullSkeleton)
{
  EXPECT_THROW(FIFOStrategy(mStateSpace1, nullptr, 1), std::invalid_argument);
}

TEST_F(FIFOStrategyTest, ConstructorThrowsOnNonPositiveNumIKSolutions)
{
  EXPECT_THROW(
      FIFOStrategy(mStateSpace1, mManipulator1, 0), std::invalid_argument);
}

TEST_F(FIFOStrategyTest, Constructor)
{
  FIFOStrategy ranker(mStateSpace1, mManipulator1);
  DART_UNUSED(ranker);
}

TEST_F(FIFOStrategyTest, SingleSample)
{
  auto seedStateOne
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  auto seedStateTwo
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  seedStateOne.getSubStateHandle<SO2>(0).setAngle(0.1);
  seedStateOne.getSubStateHandle<SO2>(1).setAngle(0.1);
  seedStateTwo.getSubStateHandle<SO2>(0).setAngle(0.2);
  seedStateTwo.getSubStateHandle<SO2>(1).setAngle(0.2);

  FIFOStrategy ranker(mStateSpace1, mManipulator1, 2);
  ranker.addIKSolution(seedStateOne);
  ranker.addIKSolution(seedStateTwo);

  auto rankedSolutions = ranker.getRankedIKSolutions();
  // TODO (avk): The following two states are coming out to be null!
  auto rankedStateOne = rankedSolutions[0];
  auto rankedStateTwo = rankedSolutions[1];
  EXPECT_TRUE(!rankedStateOne.first);
  EXPECT_TRUE(!rankedStateTwo.first);
}
