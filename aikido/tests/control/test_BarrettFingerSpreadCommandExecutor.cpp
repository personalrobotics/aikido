#include <gtest/gtest.h>
#include <aikido/control/BarrettFingerSpreadCommandExecutor.hpp>
#include <dart/dart.h>

#include <chrono>

using aikido::control::BarrettFingerSpreadCommandExecutor;
using ::dart::dynamics::Chain;
using ::dart::dynamics::ChainPtr;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

using namespace dart::dynamics;
using namespace dart::collision;

class BarrettFingerSpreadCommandExecutorTest : public testing::Test
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

    // CollisionDetector
    mCollisionDetector = FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroupAsSharedPtr();

    mPosition = 1;
    mSpreadDof = 0;
  }

protected:
  SkeletonPtr mFinger;
  ChainPtr mFingerChain;
  BodyNodePtr mBn1, mBn2, mBn3;

  CollisionDetectorPtr mCollisionDetector;
  CollisionGroupPtr mCollideWith;

  double mPosition;
  int mSpreadDof;
  static constexpr double eps = 2e-1;
};


TEST_F(BarrettFingerSpreadCommandExecutorTest, constructor_NullChain_Throws)
{
  EXPECT_THROW(BarrettFingerSpreadCommandExecutor(
    nullptr, mSpreadDof, mCollisionDetector),
    std::invalid_argument);
}

TEST_F(BarrettFingerSpreadCommandExecutorTest, constructor)
{
  EXPECT_NO_THROW(BarrettFingerSpreadCommandExecutor(
    mFingerChain, mSpreadDof, mCollisionDetector));
}

TEST_F(BarrettFingerSpreadCommandExecutorTest, constructor_NonexistingSpread_throws)
{
  int spreadDof = 3;
  EXPECT_THROW(BarrettFingerSpreadCommandExecutor(
    mFingerChain, spreadDof, mCollisionDetector),
    std::invalid_argument);
}


TEST_F(BarrettFingerSpreadCommandExecutorTest, execute_WaitOnFuture_CommandExecuted)
{
  BarrettFingerSpreadCommandExecutor executor(
    mFingerChain, mSpreadDof, mCollisionDetector);

  auto future = executor.execute(mPosition, mCollideWith);

  std::future_status status;
  auto stepTime = std::chrono::milliseconds(1);
  double stepTimeCount = stepTime.count();
  do
  {
    executor.step(stepTimeCount);
    status = future.wait_for(stepTime);
  }while(status != std::future_status::ready);

  future.get();

  EXPECT_NEAR(mPosition, 
    mBn1->getParentJoint()->getDof(mSpreadDof)->getPosition(), eps);
}

// ###### TODO 
// TEST_F(BarrettFingerSpreadCommandExecutorTest, execute_PrimalInCollision_PrimalStops_DistalContinues)
// {
//   BarrettFingerSpreadCommandExecutor executor(mFingerChain, 1, 2);

//   double step = 0.3;
//   double mimicRatio = BarrettFingerSpreadCommandExecutor::getMimicRatio();

//   double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
//   double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
//   double goalPrimal = startPrimal;
//   double goalDistal = goalDistal + step*mimicRatio;

//   executor.execute(step, true);

//   EXPECT_DOUBLE_EQ(mBn2->getParentJoint()->getDof(0)->getPosition(), goalPrimal);
//   EXPECT_DOUBLE_EQ(mBn3->getParentJoint()->getDof(0)->getPosition(), goalDistal);

// }

// TEST_F(BarrettFingerSpreadCommandExecutorTest, execute_DistalInCollisionCompletes)
// {

// }

// TEST_F(BarrettFingerSpreadCommandExecutorTest, execute_SetPointAbovePrimalUpperLimitStopsAtLimit)
// {
//   BarrettFingerSpreadCommandExecutor executor(mFingerChain, 1, 2);

//   double step = 0.7;
//   double mimicRatio = BarrettFingerSpreadCommandExecutor::getMimicRatio();

//   mFingerChain->getDof(1)->setPositionUpperLimit(0.5);

//   double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
//   double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
//   double goalPrimal = 0.5;
//   double goalDistal = goalDistal + step*mimicRatio;

//   executor.execute(step, false);

//   EXPECT_DOUBLE_EQ(goalPrimal, mBn2->getParentJoint()->getDof(0)->getPosition());
//   EXPECT_DOUBLE_EQ(goalDistal, mBn3->getParentJoint()->getDof(0)->getPosition());

// }

// TEST_F(BarrettFingerSpreadCommandExecutorTest, execute_SetPointBelowPrimalLowerLimitStopsAtLimit)
// {
//   BarrettFingerSpreadCommandExecutor executor(mFingerChain, 1, 2);

//   double step = -0.7;
//   double mimicRatio = BarrettFingerSpreadCommandExecutor::getMimicRatio();

//   mFingerChain->getDof(1)->setPositionLowerLimit(0);

//   double startPrimal = mBn2->getParentJoint()->getDof(0)->getPosition();
//   double startDistal = mBn3->getParentJoint()->getDof(0)->getPosition();
//   double goalPrimal = 0;
//   double goalDistal = goalDistal + step*mimicRatio;

//   executor.execute(step, false);

//   EXPECT_DOUBLE_EQ(goalPrimal, mBn2->getParentJoint()->getDof(0)->getPosition());
//   EXPECT_DOUBLE_EQ(goalDistal, mBn3->getParentJoint()->getDof(0)->getPosition());

// }

// TEST_F(BarrettFingerSpreadCommandExecutorTest, execute_SetPointAboveDistalUpperLimitStopsAtLimit)
// {}

// TEST_F(BarrettFingerSpreadCommandExecutorTest, execute_SetPointBelowLowerDistalLimitStopsAtLimit)
// {}