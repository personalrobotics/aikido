#include <chrono>

#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <gtest/gtest.h>

#include <aikido/control/KinematicSimulationVelocityExecutor.hpp>

using aikido::control::KinematicSimulationVelocityExecutor;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;

const static std::chrono::milliseconds waitTime{0};
const static std::chrono::milliseconds stepTime{100};

class KinematicSimulationVelocityExecutorTest : public testing::Test
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

    mCommand = std::vector<double>(2, 1.0);
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;

  BodyNodePtr bn1;

  std::vector<double> mCommand;
};

TEST_F(KinematicSimulationVelocityExecutorTest, constructor_NullSkeleton_Throws)
{
  EXPECT_THROW(
      KinematicSimulationVelocityExecutor(nullptr), std::invalid_argument);
}

TEST_F(KinematicSimulationVelocityExecutorTest, constructor_Passes)
{
  EXPECT_NO_THROW(KinematicSimulationVelocityExecutor executor(mSkeleton));
}

TEST_F(KinematicSimulationVelocityExecutorTest, execute_EmptyCommand_Throws)
{
  KinematicSimulationVelocityExecutor executor(mSkeleton);
  EXPECT_THROW(
      executor.execute(std::vector<double>()).get(), std::runtime_error);
}

TEST_F(
    KinematicSimulationVelocityExecutorTest, execute_PoseWithWrongDofs_Throws)
{
  auto skeleton = Skeleton::create("Skeleton");

  KinematicSimulationVelocityExecutor executor(skeleton);
  EXPECT_THROW(
      executor.execute(std::vector<double>(6)).get(), std::runtime_error);
}

TEST_F(
    KinematicSimulationVelocityExecutorTest,
    execute_WaitOnFuture_CommandWasExecuted)
{
  KinematicSimulationVelocityExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto future = executor.execute(mCommand);

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  EXPECT_NO_THROW(future.get());

  double timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent, 1E-4);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent, 1E-4);
}

TEST_F(
    KinematicSimulationVelocityExecutorTest,
    execute_CommandIsAlreadyRunning_NoThrows)
{
  KinematicSimulationVelocityExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto future = executor.execute(std::vector<double>(2, 0.1));

  EXPECT_NO_THROW(future = executor.execute(mCommand));

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  EXPECT_NO_THROW(future.get());

  double timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent, 1E-4);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent, 1E-4);
}

TEST_F(
    KinematicSimulationVelocityExecutorTest,
    execute_CommandFinished_DoesNotThrow)
{
  KinematicSimulationVelocityExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto future = executor.execute(mCommand);

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();

  double timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent, 1E-4);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent, 1E-4);

  mSkeleton->getDof(0)->setPosition(-1.0);
  mSkeleton->getDof(1)->setPosition(-1.0);

  // Execute second traj.
  simulationClock = std::chrono::system_clock::now();
  beginning = simulationClock;
  future = executor.execute(mCommand);

  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();

  timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent - 1.0, 1E-4);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent - 1.0, 1E-4);
}

TEST_F(KinematicSimulationVelocityExecutorTest, step_NegativeTimepoint_NoThrows)
{
  KinematicSimulationVelocityExecutor executor(mSkeleton);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
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

  double timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent, 1E-4);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent, 1E-4);
}

TEST_F(
    KinematicSimulationVelocityExecutorTest, cancel_TrajectoryInProgress_Halts)
{
  KinematicSimulationVelocityExecutor executor(mSkeleton);

  auto simulationClock = std::chrono::system_clock::now();
  auto future = executor.execute(mCommand);
  future.wait_for(waitTime);
  executor.step(simulationClock + stepTime);
  executor.cancel();

  EXPECT_THROW(future.get(), std::runtime_error);

  EXPECT_NEAR(
      mSkeleton->getDof(0)->getPosition(), stepTime.count() / 1000.0, 1E-4);
  EXPECT_NEAR(
      mSkeleton->getDof(1)->getPosition(), stepTime.count() / 1000.0, 1E-4);
}

TEST_F(KinematicSimulationVelocityExecutorTest, execute_RealTime)
{
  KinematicSimulationVelocityExecutor executor(mSkeleton);
  executor.start();

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto timeout = std::chrono::duration<double>(0.05);
  auto startClock = std::chrono::system_clock::now();
  auto future = executor.execute(mCommand, timeout);

  EXPECT_EQ(future.get(), 0);
  auto endClock = std::chrono::system_clock::now();
  auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      endClock - startClock);
  auto duration = std::chrono::duration<double>(endClock - startClock);
  EXPECT_EQ(duration_ms.count(), 50);

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), duration.count(), 1E-2);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), duration.count(), 1E-2);
  executor.stop();
}
