#include <aikido/sampleable/IKSampleable.hpp>
#include <aikido/sampleable/TSR.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::util::RNGWrapper;
using aikido::util::RNG;
using namespace aikido::sampleable;
using namespace dart::dynamics;


// Derives a fixture FooTest from BaseTest.
class IKConstraintTest : public ::testing::Test {
  protected:
    virtual void SetUp() {
  
      tsr = std::shared_ptr<TSR>(new TSR());
      rng.reset(new RNGWrapper<std::default_random_engine>(0)); 

      // Setup for constrained IKPtr 
      SkeletonPtr manipulator = Skeleton::create("manipulator");

      // Root joint
      RevoluteJoint::Properties properties1;
      properties1.mAxis = Eigen::Vector3d::UnitY();
      properties1.mName = "root joint";
      bn1 = manipulator->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, properties1, 
        BodyNode::Properties(std::string("root_body"))).second;

      // joint 2, body 2
      RevoluteJoint::Properties properties2;
      properties2.mAxis = Eigen::Vector3d::UnitY();
      properties2.mName = "joint 2";
      properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
      bn2 = manipulator->createJointAndBodyNodePair<RevoluteJoint>(
        bn1, properties2, 
        BodyNode::Properties(std::string("second_body"))).second;
      constrained_ik = InverseKinematics::create(bn2);


      // Setup for relaxed IKPtr 
      SkeletonPtr manipulator2 = Skeleton::create("manipulator");

      // Root joint
      FreeJoint::Properties properties3;
      properties3.mName = "root joint";
      bn3 = manipulator2->createJointAndBodyNodePair<FreeJoint>(
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
      properties4.mName = "joint 2";
      properties4.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
      bn4 = manipulator2->createJointAndBodyNodePair<RevoluteJoint>(
        bn3, properties4, 
        BodyNode::Properties(std::string("second_body"))).second;
      relaxed_ik = InverseKinematics::create(bn4);
    }

    std::shared_ptr<TSR> tsr;
    InverseKinematicsPtr constrained_ik, relaxed_ik;
    std::unique_ptr<RNG> rng;
    BodyNodePtr bn1, bn2, bn3, bn4;

};


TEST_F(IKConstraintTest, Constructor)
{
  IKSampleableConstraint ikConstraint(tsr, constrained_ik, std::move(rng), 5);
}

TEST_F(IKConstraintTest, CopyConstructor)
{
  IKSampleableConstraint ikConstraint(tsr, constrained_ik, std::move(rng), 5);
  IKSampleableConstraint ikConstraint2(ikConstraint);
}

TEST_F(IKConstraintTest, AssignmentOperator)
{
  IKSampleableConstraint ikConstraint(tsr, constrained_ik, std::move(rng), 5);
  IKSampleableConstraint ikConstraint2 = ikConstraint;
}

TEST_F(IKConstraintTest, SampleGeneratorPointConstraint)
{
  Eigen::Isometry3d T0_w(Eigen::Isometry3d::Identity());
  T0_w.translation() = Eigen::Vector3d(0, 0, 1);
  tsr->mT0_w = T0_w;

  IKSampleableConstraint ikConstraint(tsr, constrained_ik, std::move(rng), 5);
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator = 
    ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator->canSample());
  boost::optional<Eigen::VectorXd> sample = generator->sample();
  ASSERT_TRUE(sample);
  EXPECT_TRUE(sample.get().isZero(1e-5));
}

TEST_F(IKConstraintTest, SampleGeneratorJointLimitInfeasible)
{
  bn1->getParentJoint()->setPosition(0, M_PI/4);
  tsr->mT0_w = bn2->getTransform();

  /// Set first joint to be a fixed joint.
  bn1->getParentJoint()->setPositionLowerLimit(0, 0);
  bn1->getParentJoint()->setPositionUpperLimit(0, 0);

  IKSampleableConstraint ikConstraint(tsr, constrained_ik, std::move(rng), 5);
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

  IKSampleableConstraint ikConstraint(tsr, relaxed_ik, std::move(rng), 5);
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator1 = 
    ikConstraint.createSampleGenerator();
  std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator2 = 
    ikConstraint.createSampleGenerator();

  ASSERT_TRUE(generator1->canSample());
  ASSERT_TRUE(generator2->canSample());

  for(int i = 0; i < 10; i++)
  {
    boost::optional<Eigen::VectorXd> sample1 = generator1->sample();
    ASSERT_TRUE(sample1);
    boost::optional<Eigen::VectorXd> sample2 = generator2->sample();
    ASSERT_TRUE(sample2);
    EXPECT_EQ(sample1.get(), sample2.get());
  }
}

