#include <gtest/gtest.h>
#include <aikido/control/HandSimulationTrajectoryExecutor.hpp>
#include <aikido/control/FingerSimulationStepExecutor.hpp>
#include <dart/dart.h>

#include <chrono>

using aikido::control::HandSimulationTrajectoryExecutor;
using aikido::control::FingerSimulationStepExecutor;

using ::dart::dynamics::Chain;
using ::dart::dynamics::ChainPtr;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

using namespace dart::dynamics;
using namespace dart::collision;

class HandSimulationTrajectoryExecutorTest : public testing::Test
{
public:

  ChainPtr create3DoFFinger()
  {
    auto mFinger = Skeleton::create("Finger");

    // spread joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitZ();
    properties1.mName = "Joint1";
    auto mBn1 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties1, 
      BodyNode::Properties(std::string("spread"))).second;

    // primal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    auto mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn1, properties2, 
      BodyNode::Properties(std::string("primal"))).second;

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    auto mBn3 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn2, properties3, 
      BodyNode::Properties(std::string("distal"))).second;

    Chain::IncludeBoth_t t;
    return Chain::create(mBn1, mBn3, t);
  }

  ChainPtr create2DoFFinger()
  {
    auto mFinger = Skeleton::create("Finger");

    // primal joint
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    auto mBn2 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties2, 
      BodyNode::Properties(std::string("primal"))).second;

    // distal joint
    RevoluteJoint::Properties properties3;
    properties3.mAxis = Eigen::Vector3d::UnitY();
    properties3.mName = "Joint3";
    properties3.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    auto mBn3 = mFinger->createJointAndBodyNodePair<RevoluteJoint>(
      mBn2, properties3, 
      BodyNode::Properties(std::string("distal"))).second;

    Chain::IncludeBoth_t t;
    return Chain::create(mBn2, mBn3, t);
  }

  virtual void SetUp()
  {
    // Finger
    mFingerChains.reserve(3);
    mFingerChains.emplace_back(create3DoFFinger());
    mFingerChains.emplace_back(create3DoFFinger());
    mFingerChains.emplace_back(create2DoFFinger());
    
    // CollisionDetector
    mCollisionDetector = FCLCollisionDetector::create();
    mCollideWith = mCollisionDetector->createCollisionGroup();

    // Cycle
    mCyclePeriod = std::chrono::milliseconds(1);
    mDuration = std::chrono::milliseconds(3);

    mSetpoints.reserve(3);
    mSetpoints.emplace_back(1.0);
    mSetpoints.emplace_back(1.0);
    mSetpoints.emplace_back(1.0);

    mSpread = M_PI/2;
  }

protected:
  std::vector<ChainPtr> mFingerChains;
  std::chrono::milliseconds mCyclePeriod, mDuration;
  CollisionDetectorPtr mCollisionDetector;
  CollisionGroupPtr mCollideWith;

  std::vector<double> mSetpoints;
  double mSpread;



};

TEST_F(HandSimulationTrajectoryExecutorTest, constructor_NullChain_Throws)
{
  std::vector<ChainPtr> fingers; 
  fingers.push_back(nullptr);
  fingers.push_back(nullptr);
  fingers.push_back(nullptr);
  
  EXPECT_THROW(HandSimulationTrajectoryExecutor(fingers,
    mCyclePeriod, mCollisionDetector, mCollideWith),
    std::invalid_argument);
}

TEST_F(HandSimulationTrajectoryExecutorTest, constructor_negative_cycle_Throws)
{
  mCyclePeriod = std::chrono::milliseconds(-3);

  EXPECT_THROW(HandSimulationTrajectoryExecutor(mFingerChains,
    mCyclePeriod, mCollisionDetector, mCollideWith),
    std::invalid_argument);
}



TEST_F(HandSimulationTrajectoryExecutorTest, constructor_zero_cycle_Throws)
{
  mCyclePeriod = std::chrono::milliseconds(0);

  EXPECT_THROW(HandSimulationTrajectoryExecutor(mFingerChains,
    mCyclePeriod, mCollisionDetector, mCollideWith),
    std::invalid_argument);
}


TEST_F(HandSimulationTrajectoryExecutorTest, constructor_lessthan3Fingers_throws)
{
  std::vector<ChainPtr> fingers; 
  fingers.push_back(create3DoFFinger());
  fingers.push_back(create3DoFFinger());
  
  EXPECT_THROW(HandSimulationTrajectoryExecutor(fingers,
    mCyclePeriod, mCollisionDetector, mCollideWith),
    std::invalid_argument);
}

TEST_F(HandSimulationTrajectoryExecutorTest, constructor_incorecctDofFingers_throws)
{
  std::vector<ChainPtr> fingers; 
  fingers.push_back(create3DoFFinger());
  fingers.push_back(create2DoFFinger());
  fingers.push_back(create3DoFFinger());

  EXPECT_THROW(HandSimulationTrajectoryExecutor(fingers,
    mCyclePeriod, mCollisionDetector, mCollideWith),
    std::invalid_argument);
}

TEST_F(HandSimulationTrajectoryExecutorTest, constructor_no_throw)
{
  EXPECT_NO_THROW(HandSimulationTrajectoryExecutor(mFingerChains,
    mCyclePeriod, mCollisionDetector, mCollideWith));
}

TEST_F(HandSimulationTrajectoryExecutorTest, constructor_NullCollisionDetector_throws)
{
  EXPECT_THROW(HandSimulationTrajectoryExecutor(mFingerChains,
    mCyclePeriod, nullptr, mCollideWith), std::invalid_argument);
}

TEST_F(HandSimulationTrajectoryExecutorTest, constructor_NullCollideWith_throws)
{
  EXPECT_THROW(HandSimulationTrajectoryExecutor(mFingerChains,
    mCyclePeriod, mCollisionDetector, nullptr), std::invalid_argument);
}

TEST_F(HandSimulationTrajectoryExecutorTest, execute_lessthan3setpoints_throws)
{
  HandSimulationTrajectoryExecutor executor(mFingerChains,
    mCyclePeriod, mCollisionDetector, mCollideWith);

  std::vector<double> setpoints;
  setpoints.push_back(0.0);

  EXPECT_THROW(executor.execute(setpoints, mSpread, mDuration),
    std::invalid_argument);
}


TEST_F(HandSimulationTrajectoryExecutorTest, execute_morethan3setpoints_throws)
{
  HandSimulationTrajectoryExecutor executor(mFingerChains,
    mCyclePeriod, mCollisionDetector, mCollideWith);

  std::vector<double> setpoints;
  setpoints.push_back(0.0);
  setpoints.push_back(2.0);
  setpoints.push_back(2.0);
  setpoints.push_back(2.0);


  EXPECT_THROW(executor.execute(setpoints, mSpread, mDuration),
    std::invalid_argument);
}

TEST_F(HandSimulationTrajectoryExecutorTest, execute_negative_duration_throws)
{
  HandSimulationTrajectoryExecutor executor(mFingerChains,
    mCyclePeriod, mCollisionDetector, mCollideWith);

  auto duration = std::chrono::milliseconds(-1);

  EXPECT_THROW(executor.execute(mSetpoints, mSpread, duration),
    std::invalid_argument);
}


TEST_F(HandSimulationTrajectoryExecutorTest, execute_nocollision)
{
  // Setup
  HandSimulationTrajectoryExecutor executor(mFingerChains,
    mCyclePeriod, mCollisionDetector, mCollideWith);

  double startPrimalDofValues[3];
  double startDistalDofValues[3];
  double startSpread = mFingerChains[0]->getDof(0)->getPosition();
  double mimicRatio = FingerSimulationStepExecutor::getMimicRatio();

  for(int i = 0 ; i < 2; ++i)
  {
    startPrimalDofValues[i] = mFingerChains[i]->getDof(1)->getPosition();
  }
  startPrimalDofValues[2] = mFingerChains[2]->getDof(0)->getPosition();

  for(int i = 0 ; i < 2; ++i)
  {
    startDistalDofValues[i] = mFingerChains[i]->getDof(2)->getPosition();
  }
  startDistalDofValues[2] = mFingerChains[2]->getDof(1)->getPosition();

  // Execute trajectory
  executor.execute(mSetpoints, mSpread, mDuration);

  // Validate dof values
  for(int i = 0; i < 2; ++i)
  {
    double primal = mFingerChains[i]->getDof(1)->getPosition();
    double distal = mFingerChains[i]->getDof(2)->getPosition();

    EXPECT_DOUBLE_EQ(primal, startDistalDofValues[i] + mSetpoints[i]);
    EXPECT_DOUBLE_EQ(distal, startDistalDofValues[i] + mSetpoints[i]*mimicRatio);

    double spread = mFingerChains[i]->getDof(0)->getPosition();
    EXPECT_DOUBLE_EQ(spread, startSpread + mSpread);
  }

  double primal = mFingerChains[2]->getDof(0)->getPosition();
  double distal = mFingerChains[2]->getDof(1)->getPosition();

  EXPECT_DOUBLE_EQ(primal, startPrimalDofValues[2] + mSetpoints[2]);
  EXPECT_DOUBLE_EQ(distal, startDistalDofValues[2] + mSetpoints[2]*mimicRatio);
}







