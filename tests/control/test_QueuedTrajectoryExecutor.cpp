#include <chrono>
#include <gtest/gtest.h>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/control/QueuedTrajectoryExecutor.hpp>
#include <aikido/control/KinematicSimulationTrajectoryExecutor.hpp>
#include <aikido/statespace/GeodesicInterpolator.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/trajectory/Interpolated.hpp>

using aikido::control::TrajectoryExecutor;
using aikido::control::QueuedTrajectoryExecutor;
using aikido::control::KinematicSimulationTrajectoryExecutor;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::Interpolator;
using aikido::statespace::GeodesicInterpolator;
using aikido::statespace::SO2;
using aikido::trajectory::TrajectoryPtr;
using aikido::trajectory::Interpolated;
using ::dart::common::make_unique;
using ::dart::dynamics::Group;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;

const static std::chrono::milliseconds zeroTime{0};
const static std::chrono::milliseconds stepTime{100};

class QueuedTrajectoryExecutorTest : public testing::Test
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

    mSpace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton);
    interpolator = std::make_shared<GeodesicInterpolator>(mSpace);

    auto s1 = mSpace->getScopedStateFromMetaSkeleton();
    s1.getSubStateHandle<SO2>(0).setAngle(0);
    auto s2 = mSpace->getScopedStateFromMetaSkeleton();
    s2.getSubStateHandle<SO2>(0).setAngle(1);
    auto s3 = mSpace->getScopedStateFromMetaSkeleton();
    s3.getSubStateHandle<SO2>(0).setAngle(2);

    mTraj1 = std::make_shared<Interpolated>(mSpace, interpolator);
    mTraj1->addWaypoint(0, s1);
    mTraj1->addWaypoint(1, s2);

    mTraj2 = std::make_shared<Interpolated>(mSpace, interpolator);
    mTraj2->addWaypoint(0, s2);
    mTraj2->addWaypoint(1, s3);

    mExecutor = make_unique<KinematicSimulationTrajectoryExecutor>(mSkeleton);
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
  MetaSkeletonStateSpacePtr mSpace;

  std::shared_ptr<Interpolator> interpolator;
  std::shared_ptr<Interpolated> mTraj1;
  std::shared_ptr<Interpolated> mTraj2;

  std::unique_ptr<TrajectoryExecutor> mExecutor;

  BodyNodePtr bn1;
};

TEST_F(
    QueuedTrajectoryExecutorTest, constructor_NullExecutor_Throws)
{
  EXPECT_THROW(
      QueuedTrajectoryExecutor(nullptr), std::invalid_argument);
}

TEST_F(QueuedTrajectoryExecutorTest, constructor_Passes)
{
  EXPECT_NO_THROW(QueuedTrajectoryExecutor executor(std::move(mExecutor)));
}

TEST_F(
    QueuedTrajectoryExecutorTest,
    execute_WaitOnFuture_TrajectoryWasExecuted)
{
  QueuedTrajectoryExecutor executor(std::move(mExecutor));

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto future = executor.execute(mTraj1);

  std::future_status status;
  do
  {
    executor.step();
    status = future.wait_for(stepTime);
  } while (status != std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
}

TEST_F(
    QueuedTrajectoryExecutorTest,
    execute_QueueTrajectoryWhileRunning_BothTrajectoriesExecuted)
{
  QueuedTrajectoryExecutor executor(std::move(mExecutor));

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto f1 = executor.execute(mTraj1);
  auto f2 = executor.execute(mTraj2);

  std::future_status status;
  do
  {
    executor.step();
    status = f1.wait_for(stepTime);
  } while (status != std::future_status::ready);

  f1.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);

  do
  {
    executor.step();
    status = f2.wait_for(stepTime);
  } while (status != std::future_status::ready);

  f2.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 2.0);
}

TEST_F(
    QueuedTrajectoryExecutorTest,
    execute_QueueTrajectoryWhileRunning_BothFuturesFinished)
{
  QueuedTrajectoryExecutor executor(std::move(mExecutor));

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);

  auto f1 = executor.execute(mTraj1);
  auto f2 = executor.execute(mTraj2);

  std::future_status status;
  do
  {
    executor.step();
    status = f2.wait_for(stepTime);
  } while (status != std::future_status::ready);

  f2.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 2.0);

  EXPECT_EQ(f1.wait_for(zeroTime), std::future_status::ready);
}

