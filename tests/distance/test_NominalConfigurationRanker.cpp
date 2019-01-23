#include "aikido/distance/NominalConfigurationRanker.hpp"

#include <random>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include "eigen_tests.hpp"

static constexpr double EPS = 1e-6;

using aikido::distance::NominalConfigurationRanker;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::RevoluteJoint;

static BodyNode::Properties create_BodyNodeProperties(const std::string& name)
{
  BodyNode::Properties bodyProperties;
  bodyProperties.mName = name;
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
  EXPECT_THROW(
      NominalConfigurationRanker(nullptr, mManipulator),
      std::invalid_argument);

  EXPECT_THROW(
      NominalConfigurationRanker(mStateSpace, nullptr),
      std::invalid_argument);

  std::vector<double> negativeWeights{-1, 0};
  EXPECT_THROW(
      NominalConfigurationRanker(
          mStateSpace, mManipulator, negativeWeights, nullptr),
      std::invalid_argument);

  std::vector<double> wrongDimensionWeights{1};
  EXPECT_THROW(
      NominalConfigurationRanker(
          mStateSpace, mManipulator, wrongDimensionWeights, nullptr),
      std::invalid_argument);

  NominalConfigurationRanker rankerOne(mStateSpace, mManipulator);
  DART_UNUSED(rankerOne);

  std::vector<double> goodWeights{1, 2};
  NominalConfigurationRanker rankerTwo(
      mStateSpace, mManipulator, goodWeights, nullptr);
  DART_UNUSED(rankerTwo);
}

TEST_F(NominalConfigurationRankerTest, OrderTest)
{
  std::vector<Eigen::Vector2d> jointPositions{Eigen::Vector2d(0.3, 0.3),
                                              Eigen::Vector2d(0.1, 0.1),
                                              Eigen::Vector2d(0.2, 0.2)};

  std::vector<aikido::statespace::CartesianProduct::ScopedState> states;
  for (std::size_t i = 0; i < jointPositions.size(); ++i)
  {
    auto state = mStateSpace->createState();
    mStateSpace->convertPositionsToState(jointPositions[i], state);
    states.emplace_back(state.clone());
  }

  mManipulator->setPositions(Eigen::Vector2d(0.0, 0.0));
  NominalConfigurationRanker ranker(
      mStateSpace,
      mManipulator);
  ranker.rankConfigurations(states);

  Eigen::VectorXd rankedState(2);
  jointPositions[0] = Eigen::Vector2d(0.1, 0.1);
  jointPositions[1] = Eigen::Vector2d(0.2, 0.2);
  jointPositions[2] = Eigen::Vector2d(0.3, 0.3);
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    mStateSpace->convertStateToPositions(states[i], rankedState);
    EXPECT_EIGEN_EQUAL(rankedState, jointPositions[i], EPS);
  }
}

TEST_F(NominalConfigurationRankerTest, WeightedOrderTest)
{
  std::vector<Eigen::Vector2d> jointPositions{Eigen::Vector2d(0.2, 0.5),
                                              Eigen::Vector2d(0.1, 0.6),
                                              Eigen::Vector2d(0.5, 0.1)};

  std::vector<aikido::statespace::CartesianProduct::ScopedState> states;
  for (std::size_t i = 0; i < jointPositions.size(); ++i)
  {
    auto state = mStateSpace->createState();
    mStateSpace->convertPositionsToState(jointPositions[i], state);
    states.emplace_back(state.clone());
  }

  mManipulator->setPositions(Eigen::Vector2d(0.0, 0.0));
  std::vector<double> weights{10, 1};
  NominalConfigurationRanker ranker(
      mStateSpace,
      mManipulator,
      weights,
      mStateSpace->getScopedStateFromMetaSkeleton(mManipulator.get()));
  ranker.rankConfigurations(states);

  Eigen::VectorXd rankedState(2);
  jointPositions[0] = Eigen::Vector2d(0.1, 0.6);
  jointPositions[1] = Eigen::Vector2d(0.2, 0.5);
  jointPositions[2] = Eigen::Vector2d(0.5, 0.1);
  for (std::size_t i = 0; i < states.size(); ++i)
  {
    mStateSpace->convertStateToPositions(states[i], rankedState);
    EXPECT_EIGEN_EQUAL(rankedState, jointPositions[i], EPS);
  }
}
