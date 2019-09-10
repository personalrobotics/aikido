#include <random>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <aikido/common/RNG.hpp>
#include <aikido/constraint/CyclicSampleable.hpp>
#include <aikido/constraint/FiniteSampleable.hpp>
#include <aikido/constraint/dart/InverseKinematicsSampleable.hpp>
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/StateSpace.hpp>

#include "MockConstraints.hpp"

using aikido::common::RNG;
using aikido::common::RNGWrapper;
using aikido::constraint::CyclicSampleable;
using aikido::constraint::FiniteSampleable;
using aikido::constraint::SampleGenerator;
using aikido::constraint::dart::InverseKinematicsSampleable;
using aikido::constraint::dart::TSR;
using aikido::statespace::R2;
using aikido::statespace::SE3;
using aikido::statespace::SO2;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::FreeJoint;
using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;

static BodyNode::Properties create_BodyNodeProperties(const std::string& _name)
{
  BodyNode::Properties bodyProperties;
  bodyProperties.mName = _name;
  return bodyProperties;
}

class InverseKinematicsSampleableTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mTsr.reset(new TSR);
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

    mInverseKinematics1 = InverseKinematics::create(bn2);
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

    mInverseKinematics2 = InverseKinematics::create(bn4);
    mStateSpace2
        = std::make_shared<MetaSkeletonStateSpace>(mManipulator2.get());

    // Set FiniteSampleable to generate pose close to the actual solution.
    auto seedState
        = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
    seedState.getSubStateHandle<SO2>(0).fromAngle(0.1);
    seedState.getSubStateHandle<SO2>(1).fromAngle(0.1);
    seedConstraint
        = std::make_shared<FiniteSampleable>(mStateSpace1, seedState);
  }

  std::shared_ptr<TSR> mTsr;
  std::unique_ptr<RNG> mRng;

  SkeletonPtr mManipulator1;
  MetaSkeletonStateSpacePtr mStateSpace1;
  InverseKinematicsPtr mInverseKinematics1;
  BodyNodePtr bn1, bn2;

  SkeletonPtr mManipulator2;
  MetaSkeletonStateSpacePtr mStateSpace2;
  InverseKinematicsPtr mInverseKinematics2;
  BodyNodePtr bn3, bn4;
  std::shared_ptr<FiniteSampleable> seedConstraint;
};

TEST_F(InverseKinematicsSampleableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(
      InverseKinematicsSampleable(
          nullptr, nullptr, mTsr, seedConstraint, mInverseKinematics1, 1),
      std::invalid_argument);
}

TEST_F(
    InverseKinematicsSampleableTest, ConstructorThrowsOnNullInverseKinematics)
{
  EXPECT_THROW(
      InverseKinematicsSampleable(
          mStateSpace1, mManipulator1, mTsr, seedConstraint, nullptr, 1),
      std::invalid_argument);
}

TEST_F(InverseKinematicsSampleableTest, ConstructorThrowsOnNullPoseConstraint)
{
  EXPECT_THROW(
      InverseKinematicsSampleable(
          mStateSpace1,
          mManipulator1,
          nullptr,
          seedConstraint,
          mInverseKinematics1,
          1),
      std::invalid_argument);
}

TEST_F(InverseKinematicsSampleableTest, ConstructorThrowsOnNullSeedConstraint)
{
  EXPECT_THROW(
      InverseKinematicsSampleable(
          mStateSpace1, mManipulator1, mTsr, nullptr, mInverseKinematics1, 1),
      std::invalid_argument);
}

TEST_F(
    InverseKinematicsSampleableTest, ConstructorThrowsOnSeedConstraintMismatch)
{
  auto ss = std::make_shared<MetaSkeletonStateSpace>(mManipulator1.get());
  auto st = ss->createState();
  auto sc = std::make_shared<FiniteSampleable>(ss, st);
  EXPECT_THROW(
      InverseKinematicsSampleable(
          mStateSpace1, mManipulator1, mTsr, sc, mInverseKinematics1, 1),
      std::invalid_argument);
}

TEST_F(InverseKinematicsSampleableTest, ConstructorThrowsOnNegativeTrials)
{
  EXPECT_THROW(
      InverseKinematicsSampleable(
          mStateSpace1,
          mManipulator1,
          mTsr,
          seedConstraint,
          mInverseKinematics1,
          -1),
      std::invalid_argument);
}

TEST_F(InverseKinematicsSampleableTest, ConstructorThrowsOnInvalidSeed)
{
  // Invalid statespace for seed constraint.
  Eigen::Vector2d v(1, 0);
  R2 rvss;
  auto seedStateInvalid = rvss.createState();
  seedStateInvalid.setValue(v);

  std::shared_ptr<FiniteSampleable> invalid_seed_constraint(
      new FiniteSampleable(std::make_shared<R2>(rvss), seedStateInvalid));

  EXPECT_THROW(
      InverseKinematicsSampleable(
          mStateSpace1,
          mManipulator1,
          mTsr,
          invalid_seed_constraint,
          mInverseKinematics1,
          1),
      std::invalid_argument);
}

TEST_F(InverseKinematicsSampleableTest, Constructor)
{
  // Construct valid seed_constraint.
  auto seedStateValid = mStateSpace1->createState();
  std::shared_ptr<FiniteSampleable> valid_seed_constraint(
      new FiniteSampleable(mStateSpace1, seedStateValid));

  InverseKinematicsSampleable ikConstraint(
      mStateSpace1,
      mManipulator1,
      mTsr,
      valid_seed_constraint,
      mInverseKinematics1,
      1);
}

TEST_F(InverseKinematicsSampleableTest, SingleSampleGenerator)
{
  // Set mTSR to be a pointTSR that generates
  //  the only feasible solution for mInverseKinematics1.
  Eigen::Isometry3d T0_w(Eigen::Isometry3d::Identity());
  T0_w.translation() = Eigen::Vector3d(0, 0, 1);
  mTsr->mT0_w = T0_w;

  // Set FiniteSampleable to generate pose close to the actual solution.
  auto seedState
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  seedState.getSubStateHandle<SO2>(0).fromAngle(0.1);
  seedState.getSubStateHandle<SO2>(1).fromAngle(0.1);

  std::shared_ptr<FiniteSampleable> seedConstraint(
      new FiniteSampleable(mStateSpace1, seedState));

  // Construct InverseKinematicsSampleable
  InverseKinematicsSampleable ikConstraint(
      mStateSpace1,
      mManipulator1,
      mTsr,
      seedConstraint,
      mInverseKinematics1,
      1);

  // Get IkSampleGenerator
  auto generator = ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator->canSample());
  ASSERT_EQ(generator->getNumSamples(), 1);
  auto state
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());

  ASSERT_TRUE(generator->sample(state));
  ASSERT_NEAR(state.getSubStateHandle<SO2>(0).toAngle(), 0, 1e-5);
  ASSERT_NEAR(state.getSubStateHandle<SO2>(1).toAngle(), 0, 1e-5);

  // Cannot sample anymore.
  ASSERT_FALSE(generator->canSample());
}

TEST_F(InverseKinematicsSampleableTest, CyclicSampleGenerator)
{
  // Set mTSR to be a pointTSR that generates
  //  the only feasible solution for mInverseKinematics1.
  Eigen::Isometry3d T0_w(Eigen::Isometry3d::Identity());
  T0_w.translation() = Eigen::Vector3d(0, 0, 1);
  mTsr->mT0_w = T0_w;

  // Set CyclicSampleable to generate
  // pose close to the actual solution.
  auto seedState
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  seedState.getSubStateHandle<SO2>(0).fromAngle(0.1);
  seedState.getSubStateHandle<SO2>(1).fromAngle(0.1);

  std::shared_ptr<FiniteSampleable> finiteSampleConstraint
      = std::make_shared<FiniteSampleable>(mStateSpace1, seedState);

  std::shared_ptr<CyclicSampleable> seedConstraint(
      new CyclicSampleable(finiteSampleConstraint));

  std::shared_ptr<CyclicSampleable> tsrConstraint(new CyclicSampleable(mTsr));

  // Construct InverseKinematicsSampleable
  InverseKinematicsSampleable ikConstraint(
      mStateSpace1,
      mManipulator1,
      tsrConstraint,
      seedConstraint,
      mInverseKinematics1,
      1);

  // Get IkSampleGenerator
  auto generator = ikConstraint.createSampleGenerator();

  for (int i = 1; i < 10; ++i)
  {
    ASSERT_TRUE(generator->canSample());
    ASSERT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);
    auto state
        = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());

    ASSERT_TRUE(generator->sample(state));
    ASSERT_NEAR(state.getSubStateHandle<SO2>(0).toAngle(), 0, 1e-5);
    ASSERT_NEAR(state.getSubStateHandle<SO2>(1).toAngle(), 0, 1e-5);
  }
}

TEST_F(InverseKinematicsSampleableTest, MultipleGeneratorsSampleSameSequence)
{
  // TSR constraint will generate sequence of different points.
  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(2, 0) = 1;
  Bw(2, 1) = 3;
  mTsr->mBw = Bw;

  // Set CyclicSampleable to generate
  // pose close to the actual solution.
  auto seedState
      = mStateSpace2->getScopedStateFromMetaSkeleton(mManipulator2.get());
  Eigen::Isometry3d isometry(Eigen::Isometry3d::Identity());
  isometry.translation() = Eigen::Vector3d(0.1, 0.1, 0.1);
  seedState.getSubStateHandle<SE3>(0).setIsometry(isometry);
  seedState.getSubStateHandle<SO2>(1).fromAngle(0.1);

  std::shared_ptr<FiniteSampleable> finiteSampleConstraint
      = std::make_shared<FiniteSampleable>(mStateSpace2, seedState);

  std::shared_ptr<CyclicSampleable> seedConstraint(
      new CyclicSampleable(finiteSampleConstraint));

  InverseKinematicsSampleable ikConstraint(
      mStateSpace2,
      mManipulator2,
      mTsr,
      seedConstraint,
      mInverseKinematics2,
      1);

  // Get 2 IkSampleGenerator
  auto generator1 = ikConstraint.createSampleGenerator();
  auto generator2 = ikConstraint.createSampleGenerator();

  // Test the two generators' samples for equality.
  for (int i = 0; i < 10; i++)
  {
    ASSERT_TRUE(generator1->canSample());
    ASSERT_TRUE(generator2->canSample());

    auto state1
        = mStateSpace2->getScopedStateFromMetaSkeleton(mManipulator2.get());
    auto state2
        = mStateSpace2->getScopedStateFromMetaSkeleton(mManipulator2.get());

    ASSERT_TRUE(generator1->sample(state1));
    ASSERT_TRUE(generator2->sample(state2));

    EXPECT_TRUE(state1.getSubStateHandle<SE3>(0).getIsometry().isApprox(
        state2.getSubStateHandle<SE3>(0).getIsometry()));

    EXPECT_DOUBLE_EQ(
        state1.getSubStateHandle<SO2>(1).toAngle(),
        state2.getSubStateHandle<SO2>(1).toAngle());
  }
}

TEST_F(InverseKinematicsSampleableTest, SampleGeneratorIkInfeasible)
{
  // Tests that generator returns false when IK is infeasible.

  bn1->getParentJoint()->setPosition(0, M_PI / 4);
  mTsr->mT0_w = bn2->getTransform();

  /// Set first joint to be a fixed joint.
  bn1->getParentJoint()->setPositionLowerLimit(0, 0);
  bn1->getParentJoint()->setPositionUpperLimit(0, 0);

  // Set CyclicSampleable.
  auto seedState
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  seedState.getSubStateHandle<SO2>(0).fromAngle(0);
  seedState.getSubStateHandle<SO2>(1).fromAngle(0.1);

  std::shared_ptr<FiniteSampleable> finiteSampleConstraint
      = std::make_shared<FiniteSampleable>(mStateSpace1, seedState);

  std::shared_ptr<CyclicSampleable> seedConstraint(
      new CyclicSampleable(finiteSampleConstraint));

  InverseKinematicsSampleable ikConstraint(
      mStateSpace1,
      mManipulator1,
      mTsr,
      seedConstraint,
      mInverseKinematics1,
      1);

  auto generator = ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator->canSample());

  auto state
      = mStateSpace1->getScopedStateFromMetaSkeleton(mManipulator1.get());
  ASSERT_FALSE(generator->sample(state));
}
