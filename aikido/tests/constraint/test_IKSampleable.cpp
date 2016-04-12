#include <random>
#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/constraint/TSR.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::util::RNGWrapper;
using aikido::util::RNG;
using namespace aikido::constraint;
using namespace aikido::statespace;
using namespace dart::dynamics;


class IKConstraintTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mTsr.reset(new TSR);
    mRng.reset(new RNGWrapper<std::default_random_engine>());

    // Setup for constrained IKPtr 
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

    // Setup for relaxed IKPtr 
    mManipulator2 = Skeleton::create("Manipulator2");

    // Root joint
    FreeJoint::Properties properties3;
    properties3.mName = "Joint1";
    bn3 = mManipulator2->createJointAndBodyNodePair<FreeJoint>(
      nullptr, properties3, 
      BodyNode::Properties(std::string("root_body"))).second;
    for(int i = 0; i < 6; i ++)
    {
      bn3->getParentJoint()->setPositionLowerLimit(i, -10);  
      bn3->getParentJoint()->setPositionUpperLimit(i, 10);
    }
    
    // joint 2, body 2
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

#if 0
TEST_F(IKConstraintTest, Constructor)
{
  IkSampleableConstraint constraint(mStateSpace1, mTsr, nullptr,
    mInverseKinematics1, mRng->clone(), 1);
}

TEST_F(IKConstraintTest, SampleGeneratorPointConstraint)
{
  Eigen::Isometry3d T0_w(Eigen::Isometry3d::Identity());
  T0_w.translation() = Eigen::Vector3d(0, 0, 1);
  tsr->mT0_w = T0_w;

  IkSampleableConstraint ikConstraint(tsr, constrained_ik, std::move(rng), 5);
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator = 
    ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator->canSample());
  boost::optional<Eigen::VectorXd> sample = generator->sample();
  ASSERT_TRUE(!!sample);
  EXPECT_TRUE(sample.get().isZero(1e-5));
}

TEST_F(IKConstraintTest, SampleGeneratorJointLimitInfeasible)
{
  bn1->getParentJoint()->setPosition(0, M_PI/4);
  tsr->mT0_w = bn2->getTransform();

  /// Set first joint to be a fixed joint.
  bn1->getParentJoint()->setPositionLowerLimit(0, 0);
  bn1->getParentJoint()->setPositionUpperLimit(0, 0);

  IkSampleableConstraint ikConstraint(tsr, constrained_ik, std::move(rng), 5);
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator = 
    ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator->canSample());
  boost::optional<Eigen::VectorXd> sample = generator->sample();
  ASSERT_FALSE(sample);  
}

TEST_F(IKConstraintTest, SampleSameSequence)
{

  Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
  Bw(2, 0) = 1;
  Bw(2, 1) = 3;
  tsr->mBw = Bw;

  IkSampleableConstraint ikConstraint(tsr, relaxed_ik, std::move(rng), 5);
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator1 = 
    ikConstraint.createSampleGenerator();
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator2 = 
    ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator1->canSample());
  ASSERT_TRUE(generator2->canSample());

  for(int i = 0; i < 10; i++)
  {
    boost::optional<Eigen::VectorXd> sample1 = generator1->sample();
    ASSERT_TRUE(!!sample1);
    boost::optional<Eigen::VectorXd> sample2 = generator2->sample();
    ASSERT_TRUE(!!sample2);
    EXPECT_EQ(sample1.get(), sample2.get());
  }
}

#endif
