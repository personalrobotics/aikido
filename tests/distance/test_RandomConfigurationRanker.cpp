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

    Eigen::Vector2d defaultPosition(0.0,0.0);
    mManipulator1->setPositions(defaultPosition);
  }

  SkeletonPtr mManipulator1;
  MetaSkeletonStateSpacePtr mStateSpace1;
  BodyNodePtr bn1, bn2;
};

TEST_F(RandomConfigurationRankerTest, Constructor)
{
  auto seedStateOne
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  seedStateOne.getSubStateHandle<SO2>(0).fromAngle(0.1);
  seedStateOne.getSubStateHandle<SO2>(1).fromAngle(0.1);
  std::vector<aikido::statespace::StateSpace::State*> states;
  states.emplace_back(seedStateOne);

  EXPECT_THROW(
      RandomConfigurationRanker(nullptr, mManipulator1, states), std::invalid_argument);

  EXPECT_THROW(
      RandomConfigurationRanker(mStateSpace1, nullptr, states), std::invalid_argument);

  EXPECT_THROW(
      RandomConfigurationRanker(mStateSpace1, nullptr, states), std::invalid_argument);

  RandomConfigurationRanker ranker(mStateSpace1, mManipulator1, states);
  DART_UNUSED(ranker);
}

TEST_F(RandomConfigurationRankerTest, OrderTest)
{
  auto seedStateOne
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  auto seedStateTwo
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  seedStateOne.getSubStateHandle<SO2>(0).fromAngle(0.1);
  seedStateOne.getSubStateHandle<SO2>(1).fromAngle(0.1);
  seedStateTwo.getSubStateHandle<SO2>(0).fromAngle(0.2);
  seedStateTwo.getSubStateHandle<SO2>(1).fromAngle(0.2);

  std::vector<aikido::statespace::StateSpace::State*> states;
  states.emplace_back(seedStateOne);
  states.emplace_back(seedStateTwo);

  RandomConfigurationRanker ranker(mStateSpace1, mManipulator1, states);

  auto rankedSolutions = ranker.getRankedIKSolutions();

  auto rankedStateOne = mStateSpace1->cloneState(rankedSolutions[0].first);
  auto rankedStateTwo = mStateSpace1->cloneState(rankedSolutions[1].first);

  ASSERT_NEAR(rankedStateOne.getSubStateHandle<SO2>(0).toAngle(), 0.1, 1e-5);
  ASSERT_NEAR(rankedStateOne.getSubStateHandle<SO2>(1).toAngle(), 0.1, 1e-5);
  ASSERT_NEAR(rankedStateTwo.getSubStateHandle<SO2>(0).toAngle(), 0.2, 1e-5);
  ASSERT_NEAR(rankedStateTwo.getSubStateHandle<SO2>(1).toAngle(), 0.2, 1e-5);
}
