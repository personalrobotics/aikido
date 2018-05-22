#include "aikido/distance/NominalConfigurationRanker.hpp"

#include <random>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/StateSpace.hpp>

using aikido::distance::NominalConfigurationRanker;
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

class NominalConfigurationRankerTest : public ::testing::Test
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

TEST_F(NominalConfigurationRankerTest, Constructor)
{
  auto seedStateOne
      = mStateSpace->getScopedStateFromMetaSkeleton(mManipulator.get());
  seedStateOne.getSubStateHandle<SO2>(0).fromAngle(0.1);
  seedStateOne.getSubStateHandle<SO2>(1).fromAngle(0.1);
  std::vector<aikido::statespace::StateSpace::State*> states;
  states.emplace_back(seedStateOne);

  EXPECT_THROW(
      NominalConfigurationRanker(nullptr, mManipulator, states),
      std::invalid_argument);

  EXPECT_THROW(
      NominalConfigurationRanker(mStateSpace, nullptr, states),
      std::invalid_argument);

  EXPECT_THROW(
      NominalConfigurationRanker(mStateSpace, nullptr, states),
      std::invalid_argument);

  NominalConfigurationRanker ranker(mStateSpace, mManipulator, states);
  DART_UNUSED(ranker);
}

TEST_F(NominalConfigurationRankerTest, OrderTest)
{
  auto seedStateOne
      = mStateSpace->getScopedStateFromMetaSkeleton(mManipulator.get());
  auto seedStateTwo
      = mStateSpace->getScopedStateFromMetaSkeleton(mManipulator.get());
  auto seedStateThree
      = mStateSpace->getScopedStateFromMetaSkeleton(mManipulator.get());
  seedStateOne.getSubStateHandle<SO2>(0).fromAngle(0.3);
  seedStateOne.getSubStateHandle<SO2>(1).fromAngle(0.3);
  seedStateTwo.getSubStateHandle<SO2>(0).fromAngle(0.2);
  seedStateTwo.getSubStateHandle<SO2>(1).fromAngle(0.2);
  seedStateThree.getSubStateHandle<SO2>(0).fromAngle(2 * M_PI + 0.1);
  seedStateThree.getSubStateHandle<SO2>(1).fromAngle(2 * M_PI + 0.1);

  std::vector<aikido::statespace::StateSpace::State*> states;
  states.emplace_back(seedStateOne);
  states.emplace_back(seedStateTwo);
  states.emplace_back(seedStateThree);

  NominalConfigurationRanker ranker(mStateSpace, mManipulator, states);

  auto rankedSolutions = ranker.getRankedIKSolutions();

  auto rankedStateOne = mStateSpace->cloneState(rankedSolutions[0].first);
  auto rankedStateTwo = mStateSpace->cloneState(rankedSolutions[1].first);
  auto rankedStateThree = mStateSpace->cloneState(rankedSolutions[2].first);

  ASSERT_NEAR(rankedStateOne.getSubStateHandle<SO2>(0).toAngle(), 0.1, 1e-5);
  ASSERT_NEAR(rankedStateOne.getSubStateHandle<SO2>(1).toAngle(), 0.1, 1e-5);
  ASSERT_NEAR(rankedSolutions[0].second, 0.2, 1e-5);

  ASSERT_NEAR(rankedStateTwo.getSubStateHandle<SO2>(0).toAngle(), 0.2, 1e-5);
  ASSERT_NEAR(rankedStateTwo.getSubStateHandle<SO2>(1).toAngle(), 0.2, 1e-5);
  ASSERT_NEAR(rankedSolutions[1].second, 0.4, 1e-5);

  ASSERT_NEAR(rankedStateThree.getSubStateHandle<SO2>(0).toAngle(), 0.3, 1e-5);
  ASSERT_NEAR(rankedStateThree.getSubStateHandle<SO2>(1).toAngle(), 0.3, 1e-5);
  ASSERT_NEAR(rankedSolutions[2].second, 0.6, 1e-5);
}
