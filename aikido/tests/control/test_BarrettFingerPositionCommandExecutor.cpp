#include <gtest/gtest.h>
#include <aikido/control/BarrettFingerPositionCommandExecutor.hpp>

#include <chrono>

using aikido::control::BarrettFingerPositionCommandExecutor;
using ::dart::dynamics::Chain;
using ::dart::dynamics::ChainPtr;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

class BarrettFingerPositionCommandExecutorTest : public testing::Test
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
    mFingerChain = Chain::create(mBn1, mBn3, t);
  }

protected:
  SkeletonPtr mFinger;
  ChainPtr mFingerChain;
  BodyNodePtr mBn1, mBn2, mBn3;
};


TEST_F(BarrettFingerPositionCommandExecutorTest, constructor_NullChain_Throws)
{
  EXPECT_THROW(BarrettFingerPositionCommandExecutor(nullptr, 0, 0),
    std::invalid_argument);
}

TEST_F(BarrettFingerPositionCommandExecutorTest, constructor)
{
  EXPECT_NO_THROW(BarrettFingerPositionCommandExecutor(mFingerChain, 1, 2));
}

TEST_F(BarrettFingerPositionCommandExecutorTest, constructor_NonexistentPrimal_throws)
{
  EXPECT_THROW(BarrettFingerPositionCommandExecutor(mFingerChain, 3, 1),
     std::invalid_argument);
}


TEST_F(BarrettFingerPositionCommandExecutorTest, execute_primalAndDistal)
{
  BarrettFingerPositionCommandExecutor executor(mFingerChain, 1, 2);

  double step = 0.3;
  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = startPrimal + step;
  double goalDistal = goalDistal + step*mimicRatio;

  executor.execute(step, false);

  EXPECT_DOUBLE_EQ(mBn2->getParentJoint()->getDof(0)->getPosition(), goalPrimal);
  EXPECT_DOUBLE_EQ(mBn3->getParentJoint()->getDof(0)->getPosition(), goalDistal);

}

TEST_F(BarrettFingerPositionCommandExecutorTest, execute_distalOnly)
{
  BarrettFingerPositionCommandExecutor executor(mFingerChain, 1, 2);

  double step = 0.3;
  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = startPrimal;
  double goalDistal = goalDistal + step*mimicRatio;

  executor.execute(step, true);

  EXPECT_DOUBLE_EQ(mBn2->getParentJoint()->getDof(0)->getPosition(), goalPrimal);
  EXPECT_DOUBLE_EQ(mBn3->getParentJoint()->getDof(0)->getPosition(), goalDistal);

}

TEST_F(BarrettFingerPositionCommandExecutorTest, execute_stepAboveDofUpperLimit_mapToDofUpperLimit)
{
  BarrettFingerPositionCommandExecutor executor(mFingerChain, 1, 2);

  double step = 0.7;
  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  mFingerChain->getDof(1)->setPositionUpperLimit(0.5);

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = 0.5;
  double goalDistal = goalDistal + step*mimicRatio;

  executor.execute(step, false);

  EXPECT_DOUBLE_EQ(goalPrimal, mBn2->getParentJoint()->getDof(0)->getPosition());
  EXPECT_DOUBLE_EQ(goalDistal, mBn3->getParentJoint()->getDof(0)->getPosition());

}



TEST_F(BarrettFingerPositionCommandExecutorTest, execute_stepAboveDofLowerLimit_mapToDofLowerLimit)
{
  BarrettFingerPositionCommandExecutor executor(mFingerChain, 1, 2);

  double step = -0.7;
  double mimicRatio = BarrettFingerPositionCommandExecutor::getMimicRatio();

  mFingerChain->getDof(1)->setPositionLowerLimit(0);

  double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
  double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
  double goalPrimal = 0;
  double goalDistal = goalDistal + step*mimicRatio;

  executor.execute(step, false);

  EXPECT_DOUBLE_EQ(goalPrimal, mBn2->getParentJoint()->getDof(0)->getPosition());
  EXPECT_DOUBLE_EQ(goalDistal, mBn3->getParentJoint()->getDof(0)->getPosition());

}
