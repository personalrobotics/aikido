#include <gtest/gtest.h>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <chrono>

using aikido::control::KinematicSimulationTrajectoryExecutor;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::Interpolator;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::SO2;
using aikido::trajectory::TrajectoryPtr;
using aikido::trajectory::Interpolated;
using ::dart::dynamics::Group;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

class KinematicSimulationTrajectoryExecutorTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    mSkeleton = Skeleton::create("Skeleton");

    // root joint
    RevoluteJoint::Properties jointProperties1;
    jointProperties1.mAxis = Eigen::Vector3d::UnitY();
    jointProperties1.mName = "Joint";

    BodyNode::Properties bodyProperties1;
    bodyProperties1.mName = "root_body";

    bn1 = mSkeleton->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProperties1, bodyProperties1).second;

    // joint 2, body 2
    RevoluteJoint::Properties jointProperties2;
    jointProperties2.mAxis = Eigen::Vector3d::UnitY();
    jointProperties2.mName = "Joint2";
    jointProperties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);

    BodyNode::Properties bodyProperties2;
    bodyProperties2.mName = "second_body";

    mSkeleton->createJointAndBodyNodePair<RevoluteJoint>(
      bn1, jointProperties2, bodyProperties2);

    mSpace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton);
    interpolator = std::make_shared<GeodesicInterpolator>(mSpace);

    auto s1 = mSpace->getScopedStateFromMetaSkeleton();
    s1.getSubStateHandle<SO2>(0).setAngle(0);
    auto s2 = mSpace->getScopedStateFromMetaSkeleton();
    s2.getSubStateHandle<SO2>(0).setAngle(1);

    mTraj = std::make_shared<Interpolated>(mSpace, interpolator);
    mTraj->addWaypoint(0, s1);
    mTraj->addWaypoint(1, s2);

    mPeriod = std::chrono::milliseconds(1);

  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
  MetaSkeletonStateSpacePtr mSpace;

  std::shared_ptr<Interpolator> interpolator;
  std::shared_ptr<Interpolated> mTraj;

  std::chrono::milliseconds mPeriod;

  BodyNodePtr bn1;

};

TEST_F(KinematicSimulationTrajectoryExecutorTest, constructor_NullSkeleton_Throws)
{
  EXPECT_THROW(KinematicSimulationTrajectoryExecutor(nullptr, mPeriod),
    std::invalid_argument);
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, constructor)
{
  EXPECT_NO_THROW(
    KinematicSimulationTrajectoryExecutor executor(mSkeleton, mPeriod));
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, constructor_ZeroPeriod_Throws)
{
  std::chrono::milliseconds period(0);
  EXPECT_THROW(KinematicSimulationTrajectoryExecutor(mSkeleton, period),
    std::invalid_argument);
}


TEST_F(KinematicSimulationTrajectoryExecutorTest, constructor_NegativePeriod_Throws)
{
  std::chrono::milliseconds period(-3);
  EXPECT_THROW(KinematicSimulationTrajectoryExecutor(mSkeleton, period),
    std::invalid_argument);
}


TEST_F(KinematicSimulationTrajectoryExecutorTest, execute_NullTrajectory_Throws)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton, mPeriod);
  EXPECT_THROW(executor.execute(nullptr), std::invalid_argument);
}


TEST_F(KinematicSimulationTrajectoryExecutorTest, execute_NonMetaSkeletonStateSpace_Throws)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton, mPeriod);

  auto space = std::make_shared<SO2>();
  auto interpolator = std::make_shared<GeodesicInterpolator>(space);
  auto traj = std::make_shared<Interpolated>(space, interpolator);

  EXPECT_THROW(executor.execute(traj), std::invalid_argument);
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, execute_TrajWithUncontrolledDofs_Throws)
{
  auto skeleton = Skeleton::create("Skeleton");

  KinematicSimulationTrajectoryExecutor executor(skeleton, mPeriod);
  EXPECT_THROW(executor.execute(mTraj), std::invalid_argument);
}


TEST_F(KinematicSimulationTrajectoryExecutorTest, execute_WaitOnFuture_TrajectoryWasExecuted)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton, mPeriod);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto future = executor.execute(mTraj);

  future.wait();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, execute_TrajectoryIsAlreadyRunning_Throws)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton, mPeriod);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto future = executor.execute(mTraj);
  
  // Executor possibly not able to immediately execute the next one.
  try
  {
    executor.execute(mTraj);
  }
  catch (const std::runtime_error& e)
  {
    EXPECT_EQ(std::string(e.what()), "Another trajectory in execution.");

    // It couldn't finished the trajectory execution yet.
    EXPECT_NE(mSkeleton->getDof(0)->getPosition(), 1.0);
  };

  future.wait();
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, execute_TrajectoryFinished_DoesNotThrow)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton, mPeriod);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  
  auto future = executor.execute(mTraj);
  future.wait();
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);

  mSkeleton->getDof(0)->setPosition(-1.0);
  
  // Execute second traj.
  future = executor.execute(mTraj);
  future.wait();
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);

}
