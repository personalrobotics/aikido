#include <gtest/gtest.h>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>

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
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitY();
    properties1.mName = "Joint";
    bn1 = mSkeleton->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, properties1, 
      BodyNode::Properties(std::string("root_body"))).second;

    // joint 2, body 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitY();
    properties2.mName = "Joint2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0,0,1);
    mSkeleton->createJointAndBodyNodePair<RevoluteJoint>(
      bn1, properties2, 
      BodyNode::Properties(std::string("second_body")));

    mSpace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton);
    interpolator = std::make_shared<GeodesicInterpolator>(mSpace);

    auto s1 = mSpace->getScopedStateFromMetaSkeleton();
    s1.getSubStateHandle<SO2>(0).setAngle(0);
    auto s2 = mSpace->getScopedStateFromMetaSkeleton();
    s2.getSubStateHandle<SO2>(0).setAngle(1);

    mTraj = std::make_shared<Interpolated>(mSpace, interpolator);
    mTraj->addWaypoint(10, s1);
    mTraj->addWaypoint(11, s2);

  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
  MetaSkeletonStateSpacePtr mSpace;

  std::shared_ptr<Interpolator> interpolator;
  std::shared_ptr<Interpolated> mTraj;

  BodyNodePtr bn1;

};

TEST_F(KinematicSimulationTrajectoryExecutorTest, ConstructorThrowsOnNullSkeleton)
{
  EXPECT_THROW(KinematicSimulationTrajectoryExecutor(nullptr), std::invalid_argument);
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, Constructor)
{
  EXPECT_NO_THROW(KinematicSimulationTrajectoryExecutor executor(mSkeleton));
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, ExecuteThrowsOnNullTraj)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton);
  EXPECT_THROW(executor.execute(nullptr), std::invalid_argument);
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, ExecuteThrowsOnTrajWithUnmatchingDofs)
{
  auto skeleton = Skeleton::create("Skeleton");

  KinematicSimulationTrajectoryExecutor executor(skeleton);
  EXPECT_THROW(executor.execute(mTraj), std::invalid_argument);
}


TEST_F(KinematicSimulationTrajectoryExecutorTest, ExecuteTrajSetFuture)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto future = executor.execute(mTraj);
  future.wait();

  auto result = future.get();
  EXPECT_TRUE(!!result);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
}

TEST_F(KinematicSimulationTrajectoryExecutorTest, ExecuteTrajThrowOnMultipleImmediateExecution)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto future = executor.execute(mTraj);
  
  // Executor cannot not immediately execute the next one.
  EXPECT_THROW(executor.execute(mTraj), std::runtime_error);

  future.wait();
  auto result = future.get();
  EXPECT_TRUE(!!result);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
}


TEST_F(KinematicSimulationTrajectoryExecutorTest, WaitAndExecuteMultipleTrajectories)
{
  KinematicSimulationTrajectoryExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  
  auto future = executor.execute(mTraj);
  future.wait();
  auto result = future.get();
  EXPECT_TRUE(!!result);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);

  mSkeleton->getDof(0)->setPosition(-1.0);
  
  // Execute second traj.
  future = executor.execute(mTraj);
  result = future.get();
  EXPECT_TRUE(!!result);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);

}
