#include "aikido/distance/RandomConfigurationRanker.hpp"

#include <random>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/StateSpace.hpp>

using aikido::distance::RandomConfigurationRanker;
using aikido::statespace::SO2;
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

class RandomConfigurationRankerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Manipulator with 2 revolute joints.
    mManipulator = Skeleton::create("Manipulator");

    // Root joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitY();
    properties1.mName = "Joint1";

    bn1 = mManipulator
              ->createJointAndBodyNodePair<RevoluteJoint>(
                  nullptr, properties1, create_BodyNodeProperties("root"))
              .second;

    // joint 2, body 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, 1);
    bn2 = mManipulator
              ->createJointAndBodyNodePair<RevoluteJoint>(
                  bn1, properties2, create_BodyNodeProperties("leaf"))
              .second;

    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(mManipulator.get());

    Eigen::Vector2d defaultPosition(0.0, 0.0);
    mManipulator->setPositions(defaultPosition);
  }

  SkeletonPtr mManipulator;
  MetaSkeletonStateSpacePtr mStateSpace;
  BodyNodePtr bn1, bn2;
};

TEST_F(RandomConfigurationRankerTest, Constructor)
{
  auto seedStateOne
      = mStateSpace->getScopedStateFromMetaSkeleton(mManipulator.get());
  seedStateOne.getSubStateHandle<SO2>(0).fromAngle(0.1);
  seedStateOne.getSubStateHandle<SO2>(1).fromAngle(0.1);
  std::vector<aikido::statespace::StateSpace::State*> states;
  states.emplace_back(seedStateOne);

  EXPECT_THROW(
      RandomConfigurationRanker(nullptr, mManipulator, states),
      std::invalid_argument);

  EXPECT_THROW(
      RandomConfigurationRanker(mStateSpace, nullptr, states),
      std::invalid_argument);

  EXPECT_THROW(
      RandomConfigurationRanker(mStateSpace, nullptr, states),
      std::invalid_argument);

  RandomConfigurationRanker ranker(mStateSpace, mManipulator, states);
  DART_UNUSED(ranker);
}

TEST_F(RandomConfigurationRankerTest, OrderTest)
{
  auto seedStateOne
      = mStateSpace->getScopedStateFromMetaSkeleton(mManipulator.get());
  auto seedStateTwo
      = mStateSpace->getScopedStateFromMetaSkeleton(mManipulator.get());
  seedStateOne.getSubStateHandle<SO2>(0).fromAngle(0.1);
  seedStateOne.getSubStateHandle<SO2>(1).fromAngle(0.1);
  seedStateTwo.getSubStateHandle<SO2>(0).fromAngle(0.2);
  seedStateTwo.getSubStateHandle<SO2>(1).fromAngle(0.2);

  std::vector<aikido::statespace::StateSpace::State*> states;
  states.emplace_back(seedStateOne);
  states.emplace_back(seedStateTwo);

  RandomConfigurationRanker ranker(mStateSpace, mManipulator, states);

  auto rankedSolutions = ranker.getRankedIKSolutions();

  auto rankedStateOne = mStateSpace->cloneState(rankedSolutions[0].first);
  auto rankedStateTwo = mStateSpace->cloneState(rankedSolutions[1].first);

  ASSERT_NEAR(rankedStateOne.getSubStateHandle<SO2>(0).toAngle(), 0.1, 1e-5);
  ASSERT_NEAR(rankedStateOne.getSubStateHandle<SO2>(1).toAngle(), 0.1, 1e-5);
  ASSERT_NEAR(rankedStateTwo.getSubStateHandle<SO2>(0).toAngle(), 0.2, 1e-5);
  ASSERT_NEAR(rankedStateTwo.getSubStateHandle<SO2>(1).toAngle(), 0.2, 1e-5);
}
