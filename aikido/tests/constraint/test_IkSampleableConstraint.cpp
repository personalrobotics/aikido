#include <random>
#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/constraint/FiniteSampleConstraint.hpp>
#include <aikido/constraint/FiniteCyclicSampleConstraint.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::statespace::RealVectorStateSpace;
using aikido::statespace::SO2StateSpace;
using aikido::constraint::FiniteSampleConstraint;
using aikido::constraint::IkSampleableConstraint;
using aikido::constraint::FiniteCyclicSampleConstraint;
using aikido::statespace::SE3StateSpace;
using dart::dynamics::FreeJoint;
using aikido::constraint::SampleGenerator;
using aikido::constraint::TSR;
using aikido::util::RNGWrapper;
using aikido::util::RNG;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::InverseKinematics;
using dart::dynamics::InverseKinematicsPtr;

class IkSampleableConstraintTest : public ::testing::Test
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
    bn1 = mManipulator1->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties1, 
      BodyNode::Properties(std::string("root_body"))).second;

    // joint 2, body 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    bn2 = mManipulator1->createJointAndBodyNodePair<RevoluteJoint>(
      bn1, properties2, 
      BodyNode::Properties(std::string("second_body"))).second;

    mInverseKinematics1 = InverseKinematics::create(bn2);
    mStateSpace1 = std::make_shared<MetaSkeletonStateSpace>(mManipulator1);

    
    // Manipulator with 1 free joint and 1 revolute joint. 
    mManipulator2 = Skeleton::create("Manipulator2");

    // Root joint
    FreeJoint::Properties properties3;
    properties3.mName = "Joint1";
    bn3 = mManipulator2->createJointAndBodyNodePair<FreeJoint>(
      nullptr, properties3, 
      BodyNode::Properties(std::string("root_body"))).second;
    
    // Joint 2, body 2
    RevoluteJoint::Properties properties4;
    properties4.mAxis = Eigen::Vector3d::UnitY();
    properties4.mName = "Joint2";
    properties4.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    bn4 = mManipulator2->createJointAndBodyNodePair<RevoluteJoint>(
      bn3, properties4, 
      BodyNode::Properties(std::string("second_body"))).second;

    mInverseKinematics2 = InverseKinematics::create(bn4);
    mStateSpace2 = std::make_shared<MetaSkeletonStateSpace>(mManipulator2);

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
};


TEST_F(IkSampleableConstraintTest, Constructor)
{
  // Invalid statespace for seed constraint.
  Eigen::Vector2d v(1,0);
  RealVectorStateSpace rvss(2);
  auto seedStateInvalid = rvss.createState();
  seedStateInvalid.setValue(v);

  std::shared_ptr<FiniteSampleConstraint> invalid_seed_constraint( 
    new FiniteSampleConstraint(
      std::make_shared<RealVectorStateSpace>(rvss), seedStateInvalid));

  EXPECT_THROW(IkSampleableConstraint ikConstraint(mStateSpace1, mTsr,
    invalid_seed_constraint, mInverseKinematics1, mRng->clone(), 1),
    std::invalid_argument);


  // Construct valid seed_constraint.
  auto seedStateValid = mStateSpace1->createState();
  std::shared_ptr<FiniteSampleConstraint> valid_seed_constraint( 
    new FiniteSampleConstraint(mStateSpace1, seedStateValid));

  IkSampleableConstraint ikConstraint(mStateSpace1, mTsr,
    valid_seed_constraint, mInverseKinematics1, mRng->clone(), 1);
}


TEST_F(IkSampleableConstraintTest, SingleSampleGenerator)
{
  // Set mTSR to be a pointTSR that generates
  //  the only feasible solution for mInverseKinematics1.
  Eigen::Isometry3d T0_w(Eigen::Isometry3d::Identity());
  T0_w.translation() = Eigen::Vector3d(0, 0, 1);
  mTsr->mT0_w = T0_w;

  // Set FiniteSampleConstraint to generate pose close to the actual solution.
  auto seedState = mStateSpace1->getScopedStateFromMetaSkeleton();
  seedState.getSubStateHandle<SO2StateSpace>(0).setAngle(0.1);
  seedState.getSubStateHandle<SO2StateSpace>(1).setAngle(0.1);

  std::shared_ptr<FiniteSampleConstraint> seedConstraint( 
    new FiniteSampleConstraint(mStateSpace1, seedState));

  // Construct IkSampleableConstraint
  IkSampleableConstraint ikConstraint(mStateSpace1, mTsr,
    seedConstraint, mInverseKinematics1, mRng->clone(), 1);

  // Get IkSampleGenerator
  auto generator = ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator->canSample());
  ASSERT_EQ(generator->getNumSamples(), 1);
  auto state = mStateSpace1->getScopedStateFromMetaSkeleton();

  ASSERT_TRUE(generator->sample(state));
  ASSERT_NEAR(state.getSubStateHandle<SO2StateSpace>(0).getAngle(), 0, 1e-5);
  ASSERT_NEAR(state.getSubStateHandle<SO2StateSpace>(1).getAngle(), 0, 1e-5);

  // Cannot sample anymore.
  ASSERT_FALSE(generator->canSample());
}


TEST_F(IkSampleableConstraintTest, CyclicSampleGenerator)
{
  // Set mTSR to be a pointTSR that generates
  //  the only feasible solution for mInverseKinematics1.
  Eigen::Isometry3d T0_w(Eigen::Isometry3d::Identity());
  T0_w.translation() = Eigen::Vector3d(0, 0, 1);
  mTsr->mT0_w = T0_w;

  // Set FiniteCyclicSampleConstraint to generate
  // pose close to the actual solution.
  auto seedState = mStateSpace1->getScopedStateFromMetaSkeleton();
  seedState.getSubStateHandle<SO2StateSpace>(0).setAngle(0.1);
  seedState.getSubStateHandle<SO2StateSpace>(1).setAngle(0.1);

  std::shared_ptr<FiniteSampleConstraint> finiteSampleConstraint = 
    std::make_shared<FiniteSampleConstraint>(mStateSpace1, seedState);

  std::shared_ptr<FiniteCyclicSampleConstraint> seedConstraint( 
    new FiniteCyclicSampleConstraint(finiteSampleConstraint));

  std::shared_ptr<FiniteCyclicSampleConstraint> tsrConstraint(
    new FiniteCyclicSampleConstraint(mTsr));

  // Construct IkSampleableConstraint
  IkSampleableConstraint ikConstraint(mStateSpace1, tsrConstraint,
    seedConstraint, mInverseKinematics1, mRng->clone(), 1);

  // Get IkSampleGenerator
  auto generator = ikConstraint.createSampleGenerator();

  for(int i = 1; i < 10; ++i)
  {
    ASSERT_TRUE(generator->canSample());
    ASSERT_EQ(generator->getNumSamples(), SampleGenerator::NO_LIMIT);
    auto state = mStateSpace1->getScopedStateFromMetaSkeleton();

    ASSERT_TRUE(generator->sample(state));
    ASSERT_NEAR(state.getSubStateHandle<SO2StateSpace>(0).getAngle(), 0, 1e-5);
    ASSERT_NEAR(state.getSubStateHandle<SO2StateSpace>(1).getAngle(), 0, 1e-5);
  }
}


TEST_F(IkSampleableConstraintTest, MultipleGeneratorsSampleSameSequence)
{
  // TSR constraint will generate sequence of different points.
  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(2, 0) = 1;
  Bw(2, 1) = 3;
  mTsr->mBw = Bw;

  // Set FiniteCyclicSampleConstraint to generate
  // pose close to the actual solution.
  auto seedState = mStateSpace2->getScopedStateFromMetaSkeleton();
  Eigen::Isometry3d isometry(Eigen::Isometry3d::Identity());
  isometry.translation() = Eigen::Vector3d(0.1, 0.1, 0.1);
  seedState.getSubStateHandle<SE3StateSpace>(0).setIsometry(isometry);
  seedState.getSubStateHandle<SO2StateSpace>(1).setAngle(0.1);

  std::shared_ptr<FiniteSampleConstraint> finiteSampleConstraint = 
    std::make_shared<FiniteSampleConstraint>(mStateSpace2, seedState);

  std::shared_ptr<FiniteCyclicSampleConstraint> seedConstraint( 
    new FiniteCyclicSampleConstraint(finiteSampleConstraint));

  IkSampleableConstraint ikConstraint(mStateSpace2, mTsr, 
    seedConstraint, mInverseKinematics2, mRng->clone(), 1);

  // Get 2 IkSampleGenerator
  auto generator1 = ikConstraint.createSampleGenerator();
  auto generator2 = ikConstraint.createSampleGenerator();

  // Test the two generators' samples for equality.
  for(int i = 0; i < 10; i++)
  {
    ASSERT_TRUE(generator1->canSample());
    ASSERT_TRUE(generator2->canSample());

    auto state1 = mStateSpace2->getScopedStateFromMetaSkeleton();
    auto state2 = mStateSpace2->getScopedStateFromMetaSkeleton();

    ASSERT_TRUE(generator1->sample(state1));
    ASSERT_TRUE(generator2->sample(state2));

    EXPECT_TRUE(state1.getSubStateHandle<SE3StateSpace>(0).getIsometry().isApprox
      (state2.getSubStateHandle<SE3StateSpace>(0).getIsometry()));

    EXPECT_DOUBLE_EQ(state1.getSubStateHandle<SO2StateSpace>(1).getAngle(),
      state2.getSubStateHandle<SO2StateSpace>(1).getAngle());
  }
}


TEST_F(IkSampleableConstraintTest, SampleGeneratorIkInfeasible)
{
  // Tests that generator returns false when IK is infeasible.

  bn1->getParentJoint()->setPosition(0, M_PI/4);
  mTsr->mT0_w = bn2->getTransform();

  /// Set first joint to be a fixed joint.
  bn1->getParentJoint()->setPositionLowerLimit(0, 0);
  bn1->getParentJoint()->setPositionUpperLimit(0, 0);

  // Set FiniteCyclicSampleConstraint.
  auto seedState = mStateSpace1->getScopedStateFromMetaSkeleton();
  seedState.getSubStateHandle<SO2StateSpace>(0).setAngle(0);
  seedState.getSubStateHandle<SO2StateSpace>(1).setAngle(0.1);

  std::shared_ptr<FiniteSampleConstraint> finiteSampleConstraint = 
    std::make_shared<FiniteSampleConstraint>(mStateSpace1, seedState);

  std::shared_ptr<FiniteCyclicSampleConstraint> seedConstraint( 
    new FiniteCyclicSampleConstraint(finiteSampleConstraint));

  IkSampleableConstraint ikConstraint(mStateSpace1,
    mTsr, seedConstraint, mInverseKinematics1, mRng->clone(), 1);

  auto generator = ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator->canSample());

  auto state = mStateSpace1->getScopedStateFromMetaSkeleton();
  ASSERT_FALSE(generator->sample(state));
}
