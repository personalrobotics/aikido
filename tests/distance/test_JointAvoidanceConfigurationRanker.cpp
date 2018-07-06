#include "aikido/distance/JointAvoidanceConfigurationRanker.hpp"

#include <random>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/StateSpace.hpp>

using aikido::distance::JointAvoidanceConfigurationRanker;
using aikido::statespace::R1;
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

class JointAvoidanceConfigurationRankerTest : public ::testing::Test
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

    auto pair1 = mManipulator->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, properties1, create_BodyNodeProperties("root"));
    pair1.first->setPositionLowerLimit(0, 0.0);
    pair1.first->setPositionUpperLimit(0, 2 * M_PI);
    bn1 = pair1.second;

    // joint 2, body 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 0, 1);
    auto pair2 = mManipulator->createJointAndBodyNodePair<RevoluteJoint>(
        bn1, properties2, create_BodyNodeProperties("leaf"));

    bn2 = pair2.second;
    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(mManipulator.get());

    Eigen::Vector2d defaultPosition(0.0, 0.0);
    mManipulator->setPositions(defaultPosition);
  }

  SkeletonPtr mManipulator;
  MetaSkeletonStateSpacePtr mStateSpace;
  BodyNodePtr bn1, bn2;
};

TEST_F(JointAvoidanceConfigurationRankerTest, Constructor)
{
  EXPECT_THROW(
      JointAvoidanceConfigurationRanker(nullptr, mManipulator),
      std::invalid_argument);

  EXPECT_THROW(
      JointAvoidanceConfigurationRanker(mStateSpace, nullptr),
      std::invalid_argument);

  EXPECT_THROW(
      JointAvoidanceConfigurationRanker(mStateSpace, nullptr),
      std::invalid_argument);

  JointAvoidanceConfigurationRanker ranker(mStateSpace, mManipulator);
  DART_UNUSED(ranker);
}

TEST_F(JointAvoidanceConfigurationRankerTest, OrderTest)
{
  Eigen::VectorXd jointPosition(2);

  jointPosition << 0.3, 0.1;
  mManipulator->setPositions(jointPosition);
  auto seedStateOne = mStateSpace->createState();
  mStateSpace->convertPositionsToState(
      mManipulator->getPositions(), seedStateOne);

  jointPosition << 0.1, 0.4;
  mManipulator->setPositions(jointPosition);
  auto seedStateTwo = mStateSpace->createState();
  mStateSpace->convertPositionsToState(
      mManipulator->getPositions(), seedStateTwo);

  jointPosition << 0.2, 0.1;
  mManipulator->setPositions(jointPosition);
  auto seedStateThree = mStateSpace->createState();
  mStateSpace->convertPositionsToState(
      mManipulator->getPositions(), seedStateThree);

  std::vector<aikido::statespace::CartesianProduct::State*> states;
  states.emplace_back(seedStateOne);
  states.emplace_back(seedStateTwo);
  states.emplace_back(seedStateThree);

  JointAvoidanceConfigurationRanker ranker(mStateSpace, mManipulator);
  ranker.rankConfigurations(states);

  Eigen::VectorXd rankedStateOne(2), rankedStateTwo(2), rankedStateThree(2);
  mStateSpace->convertStateToPositions(
      mStateSpace->cloneState(states[0]), rankedStateOne);
  mStateSpace->convertStateToPositions(
      mStateSpace->cloneState(states[1]), rankedStateTwo);
  mStateSpace->convertStateToPositions(
      mStateSpace->cloneState(states[2]), rankedStateThree);

  EXPECT_TRUE(rankedStateOne.isApprox(Eigen::Vector2d(0.3, 0.1)));
  EXPECT_TRUE(rankedStateTwo.isApprox(Eigen::Vector2d(0.2, 0.1)));
  EXPECT_TRUE(rankedStateThree.isApprox(Eigen::Vector2d(0.1, 0.4)));
}
