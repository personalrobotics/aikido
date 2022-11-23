#include <chrono>

#include <dart/dart.hpp>
#include <gtest/gtest.h>

#include <aikido/control/InstantaneousTrajectoryExecutor.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>

using aikido::control::InstantaneousTrajectoryExecutor;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::Interpolator;
using aikido::statespace::SO2;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::trajectory::Interpolated;
using aikido::trajectory::TrajectoryPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::Group;
using ::dart::dynamics::RevoluteJoint;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;

const static std::chrono::milliseconds waitTime{0};

class InstantaneousTrajectoryExecutorTest : public testing::Test
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

    bn1 = mSkeleton
              ->createJointAndBodyNodePair<RevoluteJoint>(
                  nullptr, jointProperties1, bodyProperties1)
              .second;

    // joint 2, body 2
    RevoluteJoint::Properties jointProperties2;
    jointProperties2.mAxis = Eigen::Vector3d::UnitY();
    jointProperties2.mName = "Joint2";
    jointProperties2.mT_ParentBodyToJoint.translation()
        = Eigen::Vector3d(0, 0, 1);

    BodyNode::Properties bodyProperties2;
    bodyProperties2.mName = "second_body";

    mSkeleton->createJointAndBodyNodePair<RevoluteJoint>(
        bn1, jointProperties2, bodyProperties2);

    mSpace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton.get());
    interpolator = std::make_shared<GeodesicInterpolator>(mSpace);

    auto s1 = mSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());
    s1.getSubStateHandle<SO2>(0).fromAngle(0);
    auto s2 = mSpace->getScopedStateFromMetaSkeleton(mSkeleton.get());
    s2.getSubStateHandle<SO2>(0).fromAngle(1);

    mTraj = std::make_shared<Interpolated>(mSpace, interpolator);
    mTraj->addWaypoint(0, s1);
    mTraj->addWaypoint(1, s2);
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
  MetaSkeletonStateSpacePtr mSpace;

  std::shared_ptr<Interpolator> interpolator;
  std::shared_ptr<Interpolated> mTraj;

  BodyNodePtr bn1;
};

TEST_F(InstantaneousTrajectoryExecutorTest, constructor_NullSkeleton_Throws)
{
  EXPECT_THROW(InstantaneousTrajectoryExecutor(nullptr), std::invalid_argument);
}

TEST_F(InstantaneousTrajectoryExecutorTest, constructor_Passes)
{
  EXPECT_NO_THROW(InstantaneousTrajectoryExecutor executor(mSkeleton));
}

TEST_F(InstantaneousTrajectoryExecutorTest, execute_NullTrajectory_Throws)
{
  InstantaneousTrajectoryExecutor executor(mSkeleton);
  EXPECT_THROW(executor.execute(nullptr), std::invalid_argument);
}

TEST_F(
    InstantaneousTrajectoryExecutorTest,
    execute_NonMetaSkeletonStateSpace_Throws)
{
  InstantaneousTrajectoryExecutor executor(mSkeleton);

  auto space = std::make_shared<SO2>();
  auto interpolator = std::make_shared<GeodesicInterpolator>(space);
  auto traj = std::make_shared<Interpolated>(space, interpolator);

  EXPECT_THROW(executor.execute(traj), std::invalid_argument);
}

TEST_F(
    InstantaneousTrajectoryExecutorTest,
    execute_TrajWithUncontrolledDofs_Throws)
{
  auto skeleton = Skeleton::create("Skeleton");

  InstantaneousTrajectoryExecutor executor(skeleton);
  EXPECT_THROW(executor.execute(mTraj), std::invalid_argument);
}

TEST_F(
    InstantaneousTrajectoryExecutorTest,
    execute_WaitOnFuture_TrajectoryWasExecutedInstantly)
{
  InstantaneousTrajectoryExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto future = executor.execute(mTraj);
  std::future_status status = future.wait_for(waitTime);
  EXPECT_EQ(status, std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
}

TEST_F(
    InstantaneousTrajectoryExecutorTest,
    execute_TrajectoryFinished_DoesNotThrow)
{
  InstantaneousTrajectoryExecutor executor(mSkeleton);
  executor.start();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto future = executor.execute(mTraj);
  std::future_status status = future.wait_for(waitTime);
  EXPECT_EQ(status, std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);

  mSkeleton->getDof(0)->setPosition(0.0);

  // Execute second traj.
  future = executor.execute(mTraj);
  status = future.wait_for(waitTime);
  EXPECT_EQ(status, std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
  executor.stop();
}
