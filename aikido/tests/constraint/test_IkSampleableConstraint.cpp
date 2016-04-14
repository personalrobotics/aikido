#include <random>
#include <aikido/constraint/IkSampleableConstraint.hpp>
#include <aikido/constraint/FiniteSampleConstraint.hpp>

#include <aikido/statespace/StateSpace.hpp>

#include <aikido/constraint/TSR.hpp>
#include <aikido/util/RNG.hpp>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using aikido::util::RNGWrapper;
using aikido::util::RNG;

using namespace aikido::constraint;
using namespace aikido::statespace;
using namespace dart::dynamics;


class IkSampleableConstraintTest : public ::testing::Test
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

    // // Setup for relaxed IKPtr 
    // mManipulator2 = Skeleton::create("Manipulator2");

    // // Root joint
    // FreeJoint::Properties properties3;
    // properties3.mName = "Joint1";
    // bn3 = mManipulator2->createJointAndBodyNodePair<FreeJoint>(
    //   nullptr, properties3, 
    //   BodyNode::Properties(std::string("root_body"))).second;
    // for(int i = 0; i < 6; i ++)
    // {
    //   bn3->getParentJoint()->setPositionLowerLimit(i, -10);  
    //   bn3->getParentJoint()->setPositionUpperLimit(i, 10);
    // }
    
    // // joint 2, body 2
    // RevoluteJoint::Properties properties4;
    // properties4.mAxis = Eigen::Vector3d::UnitY();
    // properties4.mName = "Joint2";
    // properties4.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    // bn4 = mManipulator2->createJointAndBodyNodePair<RevoluteJoint>(
    //   bn3, properties4, 
    //   BodyNode::Properties(std::string("second_body"))).second;

    // mInverseKinematics2 = InverseKinematics::create(bn4);
    // mStateSpace2 = std::make_shared<MetaSkeletonStateSpace>(mManipulator2);

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



// TEST_F(IkSampleableConstraintTest, SampleGenerator)
// {
//   auto seedState = mStateSpace1->getScopedStateFromMetaSkeleton();
//   seedState.getSubStateHandle<SO2StateSpace>(0).setAngle(0);
//   seedState.getSubStateHandle<SO2StateSpace>(1).setAngle(0);

//   std::shared_ptr<FiniteSampleConstraint> seedConstraint( 
//     new FiniteSampleConstraint(mStateSpace1, seedState));

//   // Construct IkSampleableConstraint
//   IkSampleableConstraint ikConstraint(mStateSpace1, mTsr,
//     seedConstraint, mInverseKinematics1, mRng->clone(), 1);

//   // Get IkSampleGenerator
//   auto sampleGenerator = ikConstraint->createSampleGenerator();

//   ASSERT_TRUE(generator->canSample());
//   auto sampleState = mStateSpace1->createState();

//   ASSERT_TRUE(generator->sample(sampleState));
//   EXPECT_DOUBLE_EQ(state.getSubStateHandle<SO2StateSpace>(0).getAngle(), 0);
//   EXPECT_DOUBLE_EQ(state.getSubStateHandle<SO2StateSpace>(1).getAngle(), 0);



// }




// TEST_F(IkSampleableConstraintTest, PointConstraint)
// {
//   Eigen::Isometry3d T0_w(Eigen::Isometry3d::Identity());
//   T0_w.translation() = Eigen::Vector3d(0, 0, 1);
//   mTsr->mT0_w = T0_w;

//   IkSampleableConstraint constraint(mStateSpace1, mTsr, nullptr,
//     mInverseKinematics1, mRng->clone(), 1);
//   std::unique_ptr<SampleGenerator> generator = constraint.createSampleGenerator();

//   ASSERT_TRUE(generator->canSample());
//   auto state = mStateSpace1->createState();

//   ASSERT_TRUE(generator->sample(state));
//   EXPECT_DOUBLE_EQ(state.getSubStateHandle<SO2StateSpace>(0).getAngle(), 0);
//   EXPECT_DOUBLE_EQ(state.getSubStateHandle<SO2StateSpace>(1).getAngle(), 0);
// }

// TEST_F(IkSampleableConstraintTest, SampleGeneratorJointLimitInfeasible)
// {
//   bn1->getParentJoint()->setPosition(0, M_PI/4);
//   tsr->mT0_w = bn2->getTransform();

//   /// Set first joint to be a fixed joint.
//   bn1->getParentJoint()->setPositionLowerLimit(0, 0);
//   bn1->getParentJoint()->setPositionUpperLimit(0, 0);

//   IkSampleableConstraint ikConstraint(tsr, constrained_ik, std::move(rng), 5);
//   std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator = 
//     ikConstraint.createSampleGenerator();

//   ASSERT_TRUE(generator->canSample());
//   boost::optional<Eigen::VectorXd> sample = generator->sample();
//   ASSERT_FALSE(sample);  
// }

// TEST_F(IkSampleableConstraintTest, SampleSameSequence)
// {

//   Eigen::MatrixXd Bw = Eigen::Matrix<double, 6, 2>::Zero();
//   Bw(2, 0) = 1;
//   Bw(2, 1) = 3;
//   tsr->mBw = Bw;

//   IkSampleableConstraint ikConstraint(tsr, relaxed_ik, std::move(rng), 5);
//   std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator1 = 
//     ikConstraint.createSampleGenerator();
//   std::unique_ptr<SampleGenerator<Eigen::VectorXd>> generator2 = 
//     ikConstraint.createSampleGenerator();

//   ASSERT_TRUE(generator1->canSample());
//   ASSERT_TRUE(generator2->canSample());

//   for(int i = 0; i < 10; i++)
//   {
//     boost::optional<Eigen::VectorXd> sample1 = generator1->sample();
//     ASSERT_TRUE(!!sample1);
//     boost::optional<Eigen::VectorXd> sample2 = generator2->sample();
//     ASSERT_TRUE(!!sample2);
//     EXPECT_EQ(sample1.get(), sample2.get());
//   }
// }

