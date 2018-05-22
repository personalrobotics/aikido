#include "aikido/distance/NominalConfigurationRanker.hpp"

#include <random>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <aikido/common/RNG.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/StateSpace.hpp>

using aikido::statespace::R2;
using aikido::distance::NominalConfigurationRanker;
using aikido::statespace::SE3;
using aikido::statespace::SO2;
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

class NominalConfigurationRankerTest : public ::testing::Test
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

    Eigen::Vector2d defaultPosition(0.0,0.0);
    mManipulator1->setPositions(defaultPosition);
  }

  std::unique_ptr<RNG> mRng;

  SkeletonPtr mManipulator1;
  MetaSkeletonStateSpacePtr mStateSpace1;
  BodyNodePtr bn1, bn2;
};

TEST_F(NominalConfigurationRankerTest, Constructor)
{
  auto seedStateOne
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  seedStateOne.getSubStateHandle<SO2>(0).fromAngle(0.1);
  seedStateOne.getSubStateHandle<SO2>(1).fromAngle(0.1);
  std::vector<aikido::statespace::StateSpace::State*> states;
  states.emplace_back(seedStateOne);

  EXPECT_THROW(
      NominalConfigurationRanker(nullptr, mManipulator1, states), std::invalid_argument);

  EXPECT_THROW(
      NominalConfigurationRanker(mStateSpace1, nullptr, states), std::invalid_argument);

  EXPECT_THROW(
      NominalConfigurationRanker(mStateSpace1, nullptr, states), std::invalid_argument);

  NominalConfigurationRanker ranker(mStateSpace1, mManipulator1, states);
  DART_UNUSED(ranker);
}

TEST_F(NominalConfigurationRankerTest, OrderTest)
{
  auto seedStateOne
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  auto seedStateTwo
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  auto seedStateThree
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  seedStateOne.getSubStateHandle<SO2>(0).fromAngle(0.3);
  seedStateOne.getSubStateHandle<SO2>(1).fromAngle(0.3);
  seedStateTwo.getSubStateHandle<SO2>(0).fromAngle(0.2);
  seedStateTwo.getSubStateHandle<SO2>(1).fromAngle(0.2);
  seedStateThree.getSubStateHandle<SO2>(0).fromAngle(2*M_PI + 0.1);
  seedStateThree.getSubStateHandle<SO2>(1).fromAngle(2*M_PI + 0.1);

  std::vector<aikido::statespace::StateSpace::State*> states;
  states.emplace_back(seedStateOne);
  states.emplace_back(seedStateTwo);
  states.emplace_back(seedStateThree);

  NominalConfigurationRanker ranker(mStateSpace1, mManipulator1, states);

  auto rankedSolutions = ranker.getRankedIKSolutions();

  auto rankedStateOne = mStateSpace1->cloneState(rankedSolutions[0].first);
  auto rankedStateTwo = mStateSpace1->cloneState(rankedSolutions[1].first);
  auto rankedStateThree = mStateSpace1->cloneState(rankedSolutions[2].first);

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
