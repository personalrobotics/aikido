#include "aikido/distance/JointAvoidanceConfigurationRanker.hpp"

#include <random>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include "eigen_tests.hpp"

using aikido::distance::JointAvoidanceConfigurationRanker;
using aikido::statespace::R1;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::RevoluteJoint;

static constexpr double EPS = 1e-6;

static BodyNode::Properties create_BodyNodeProperties(const std::string& name)
{
  BodyNode::Properties bodyProperties;
  bodyProperties.mName = name;
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
  std::vector<Eigen::Vector2d> jointPositions{Eigen::Vector2d(0.3, 0.1),
                                              Eigen::Vector2d(0.1, 0.4),
                                              Eigen::Vector2d(0.2, 0.1)};

  std::vector<aikido::statespace::CartesianProduct::ScopedState> states;
  for (std::size_t i = 0; i < jointPositions.size(); ++i)
  {
    auto state = mStateSpace->createState();
    mStateSpace->convertPositionsToState(jointPositions[i], state);
    states.emplace_back(state.clone());
  }

  JointAvoidanceConfigurationRanker ranker(mStateSpace, mManipulator);
  ranker.rankConfigurations(states);

  Eigen::VectorXd rankedState(2);
  jointPositions[0] = Eigen::Vector2d(0.3, 0.1);
  jointPositions[1] = Eigen::Vector2d(0.2, 0.1);
  jointPositions[2] = Eigen::Vector2d(0.1, 0.4);
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    mStateSpace->convertStateToPositions(states[i], rankedState);
    EXPECT_EIGEN_EQUAL(rankedState, jointPositions[i], EPS);
  }
}
