#include <gtest/gtest.h>
#include <aikido/control/FingerSimulationStepExecutor.hpp>

#include <chrono>

using aikido::control::FingerSimulationStepExecutor;
using ::dart::dynamics::Chain;
using ::dart::dynamics::ChainPtr;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

class FingerSimulationStepExecutorTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    mFinger = Skeleton::create("Finger");

    // spread joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitZ();
    properties1.mName = "Joint1";
    mBn1 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties1, 
      BodyNode::Properties(std::string("spread"))).second;

    // primal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn1, properties2, 
      BodyNode::Properties(std::string("primal"))).second;

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    mBn3 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn2, properties3, 
      BodyNode::Properties(std::string("distal"))).second;

    Chain::IncludeBoth_t t;
    mFingerChain = Chain::create(mBn2, mBn3, t);
  }

protected:
  SkeletonPtr mFinger;
  ChainPtr mFingerChain;
  BodyNodePtr mBn1, mBn2, mBn3;
};


TEST_F(FingerSimulationStepExecutorTest, constructor_NullChain_Throws)
{
  EXPECT_THROW(FingerSimulationStepExecutor(nullptr),
    std::invalid_argument);
}

TEST_F(FingerSimulationStepExecutorTest, constructor)
{
  EXPECT_NO_THROW(FingerSimulationStepExecutor executor(mFingerChain));
}

TEST_F(FingerSimulationStepExecutorTest, constructor_ThreeDofChain_Throws)
{
  Chain::IncludeBoth_t t;
  auto finger = Chain::create(mBn1, mBn3, t);
  EXPECT_THROW(FingerSimulationStepExecutor executor(finger),
     std::invalid_argument);
}


TEST_F(FingerSimulationStepExecutorTest, execute_primalAndDistal)
{
  FingerSimulationStepExecutor executor(mFingerChain);

  double step = 0.3;
  double mimicRatio = FingerSimulationStepExecutor::getMimicRatio();

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = startPrimal + step;
  double goalDistal = goalDistal + step*mimicRatio;

  executor.execute(step, false);

  EXPECT_DOUBLE_EQ(mBn2->getParentJoint()->getDof(0)->getPosition(), goalPrimal);
  EXPECT_DOUBLE_EQ(mBn3->getParentJoint()->getDof(0)->getPosition(), goalDistal);

}

TEST_F(FingerSimulationStepExecutorTest, execute_distalOnly)
{
  FingerSimulationStepExecutor executor(mFingerChain);

  double step = 0.3;
  double mimicRatio = FingerSimulationStepExecutor::getMimicRatio();

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = startPrimal;
  double goalDistal = goalDistal + step*mimicRatio;

  executor.execute(step, true);

  EXPECT_DOUBLE_EQ(mBn2->getParentJoint()->getDof(0)->getPosition(), goalPrimal);
  EXPECT_DOUBLE_EQ(mBn3->getParentJoint()->getDof(0)->getPosition(), goalDistal);

}

TEST_F(FingerSimulationStepExecutorTest, execute_stepAboveDofUpperLimit_mapToDofUpperLimit)
{
  FingerSimulationStepExecutor executor(mFingerChain);

  double step = 0.7;
  double mimicRatio = FingerSimulationStepExecutor::getMimicRatio();

  mFingerChain->getDof(0)->setPositionUpperLimit(0.5);

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = 0.5;
  double goalDistal = goalDistal + step*mimicRatio;

  executor.execute(step, false);

  EXPECT_DOUBLE_EQ(mBn2->getParentJoint()->getDof(0)->getPosition(), goalPrimal);
  EXPECT_DOUBLE_EQ(mBn3->getParentJoint()->getDof(0)->getPosition(), goalDistal);

}



TEST_F(FingerSimulationStepExecutorTest, execute_stepAboveDofLowerLimit_mapToDofLowerLimit)
{
  FingerSimulationStepExecutor executor(mFingerChain);

  double step = -0.7;
  double mimicRatio = FingerSimulationStepExecutor::getMimicRatio();

  mFingerChain->getDof(0)->setPositionLowerLimit(0);

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = 0;
  double goalDistal = goalDistal + step*mimicRatio;

  executor.execute(step, false);

  EXPECT_DOUBLE_EQ(mBn2->getParentJoint()->getDof(0)->getPosition(), goalPrimal);
  EXPECT_DOUBLE_EQ(mBn3->getParentJoint()->getDof(0)->getPosition(), goalDistal);

}
