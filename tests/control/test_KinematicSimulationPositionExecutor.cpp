#include <chrono>

#include <gtest/gtest.h>

#include <aikido/control/KinematicSimulationPositionExecutor.hpp>

#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>

using aikido::control::KinematicSimulationPositionExecutor;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;

const static std::chrono::milliseconds waitTime{0};
const static std::chrono::milliseconds stepTime{100};

class KinematicSimulationPositionExecutorTest : public testing::Test
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

    mCommand = {1.0, 1.0};
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;

  BodyNodePtr bn1;

  std::vector<double> mCommand;
};

TEST_F(
    KinematicSimulationPositionExecutorTest, constructor_NullSkeleton_Throws)
{
  EXPECT_THROW(
      KinematicSimulationPositionExecutor(nullptr), std::invalid_argument);
}

TEST_F(KinematicSimulationPositionExecutorTest, constructor_Passes)
{
  EXPECT_NO_THROW(KinematicSimulationPositionExecutor executor(mSkeleton));
}

TEST_F(KinematicSimulationPositionExecutorTest, execute_EmptyCommand_Throws)
{
  KinematicSimulationPositionExecutor executor(mSkeleton);
  EXPECT_THROW(executor.execute(std::vector<double>()), std::runtime_error);
}

TEST_F(
    KinematicSimulationPositionExecutorTest,
    execute_PoseWithWrongDofs_Throws)
{
  auto skeleton = Skeleton::create("Skeleton");

  KinematicSimulationPositionExecutor executor(skeleton);
  EXPECT_THROW(executor.execute(std::vector<double>(6)), std::invalid_argument);
}

TEST_F(
    KinematicSimulationPositionExecutorTest,
    execute_WaitOnFuture_CommandWasExecuted)
{
  KinematicSimulationPositionExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto future = executor.execute(mCommand);

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 1.0);
}

TEST_F(
    KinematicSimulationPositionExecutorTest,
    execute_CommandIsAlreadyRunning_Throws)
{
  KinematicSimulationPositionExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto future = executor.execute(mCommand);

  EXPECT_THROW(executor.execute(mCommand).get(), std::runtime_error);
  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 1.0);
}

TEST_F(
    KinematicSimulationPositionExecutorTest,
    execute_CommandFinished_DoesNotThrow)
{
  KinematicSimulationPositionExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto future = executor.execute(mCommand);

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 1.0);

  mSkeleton->getDof(0)->setPosition(-1.0);
  mSkeleton->getDof(1)->setPosition(-1.0);

  // Execute second traj.
  simulationClock = std::chrono::system_clock::now();
  future = executor.execute(mCommand);

  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 1.0);
}

TEST_F(
    KinematicSimulationPositionExecutorTest, step_NegativeTimepoint_NoThrows)
{
  KinematicSimulationPositionExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 1.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto future = executor.execute(mCommand);

  EXPECT_NO_THROW(executor.step(simulationClock - stepTime));

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 1.0);
}

TEST_F(
    KinematicSimulationPositionExecutorTest,
    cancel_TrajectoryInProgress_Halts)
{
  KinematicSimulationPositionExecutor executor(mSkeleton);

  auto simulationClock = std::chrono::system_clock::now();
  auto future = executor.execute(mCommand);
  future.wait_for(waitTime);
  executor.step(simulationClock + stepTime);
  executor.cancel();

  EXPECT_THROW(future.get(), std::runtime_error);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), stepTime.count() / 1000.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), stepTime.count() / 1000.0);
}
