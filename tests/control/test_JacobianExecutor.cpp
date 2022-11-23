#include <chrono>

#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <gtest/gtest.h>

#include <aikido/control/JacobianExecutor.hpp>

using aikido::control::JacobianVelocityExecutor;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::PrismaticJoint;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;

const static std::chrono::milliseconds waitTime{0};
const static std::chrono::milliseconds stepTime{100};

class JacobianExecutorTest : public testing::Test
{
public:
  virtual void SetUp()
  {
    mSkeleton = Skeleton::create("Skeleton");

    // root joint
    PrismaticJoint::Properties jointProperties1;
    jointProperties1.mAxis = Eigen::Vector3d::UnitX();
    jointProperties1.mName = "XJoint";

    BodyNode::Properties bodyProperties1;
    bodyProperties1.mName = "root_body";

    auto bn1 = mSkeleton
                   ->createJointAndBodyNodePair<PrismaticJoint>(
                       nullptr, jointProperties1, bodyProperties1)
                   .second;

    // joint 2, body 2
    PrismaticJoint::Properties jointProperties2;
    jointProperties2.mAxis = Eigen::Vector3d::UnitY();
    jointProperties2.mName = "YJoint";

    BodyNode::Properties bodyProperties2;
    bodyProperties2.mName = "second_body";

    mEENode = mSkeleton
                  ->createJointAndBodyNodePair<PrismaticJoint>(
                      bn1, jointProperties2, bodyProperties2)
                  .second;

    mCommand = std::vector<double>(2, 1.0);
    mSE3Command << 0.0, 0.0, 0.0, 1.0, 1.0, 0.0;
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
  ::dart::dynamics::BodyNode* mEENode;
  std::vector<double> mCommand;
  Eigen::Vector6d mSE3Command;
};

TEST_F(JacobianExecutorTest, constructor_NullSkeleton_Throws)
{
  EXPECT_THROW(JacobianVelocityExecutor(nullptr), std::invalid_argument);
}

TEST_F(JacobianExecutorTest, constructor_Passes)
{
  EXPECT_NO_THROW(JacobianVelocityExecutor executor(mEENode));
}

TEST_F(JacobianExecutorTest, execute_EmptyCommand_Throws)
{
  JacobianVelocityExecutor executor(mEENode);
  EXPECT_THROW(
      executor.execute(std::vector<double>()).get(), std::runtime_error);
}

TEST_F(JacobianExecutorTest, execute_PoseWithWrongDofs_Throws)
{
  auto skeleton = Skeleton::create("Skeleton");

  JacobianVelocityExecutor executor(mEENode);
  EXPECT_THROW(
      executor.execute(std::vector<double>(6)).get(), std::runtime_error);
}

TEST_F(JacobianExecutorTest, execute_WaitOnFuture_CommandWasExecuted)
{
  JacobianVelocityExecutor executor(mEENode);

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

TEST_F(JacobianExecutorTest, execute_CommandIsAlreadyRunning_NoThrows)
{
  JacobianVelocityExecutor executor(mEENode);

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

TEST_F(JacobianExecutorTest, execute_CommandFinished_DoesNotThrow)
{
  JacobianVelocityExecutor executor(mEENode);

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

TEST_F(JacobianExecutorTest, step_NegativeTimepoint_NoThrows)
{
  JacobianVelocityExecutor executor(mEENode);

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

TEST_F(JacobianExecutorTest, cancel_TrajectoryInProgress_Halts)
{
  JacobianVelocityExecutor executor(mEENode);

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

TEST_F(JacobianExecutorTest, execute_WaitOnFuture_JacobianZeroCommand)
{
  JacobianVelocityExecutor executor(mEENode);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto future = executor.execute(Eigen::Vector6d::Zero());

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  EXPECT_NO_THROW(future.get());

  // Note: Jacobian math introduces 1% error.
  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), 0.0, 2E-2);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), 0.0, 2E-2);
}

TEST_F(JacobianExecutorTest, execute_WaitOnFuture_JacobianCommandWasExecuted)
{
  JacobianVelocityExecutor executor(mEENode);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto future = executor.execute(mSE3Command);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

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

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent, 2E-2);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent, 2E-2);
}

TEST_F(JacobianExecutorTest, execute_WaitOnFuture_JacobianSingularityNoThrow)
{
  JacobianVelocityExecutor executor(mEENode);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  Eigen::Vector6d zCommand;
  zCommand << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  auto future = executor.execute(zCommand);

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  EXPECT_NO_THROW(future.get());

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), 0.0, 2E-2);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), 0.0, 2E-2);
}

TEST_F(JacobianExecutorTest, execute_JacobianCommandIsAlreadyRunning_NoThrows)
{
  JacobianVelocityExecutor executor(mEENode);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto future = executor.execute(mSE3Command);

  EXPECT_NO_THROW(future = executor.execute(mSE3Command));

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

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent, 2E-2);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent, 2E-2);
}

TEST_F(
    JacobianExecutorTest,
    execute_JacobianCommandIsAlreadyRunning_JointCommandNoThrows)
{
  JacobianVelocityExecutor executor(mEENode);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto future = executor.execute(mSE3Command);

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

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent, 2E-2);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent, 2E-2);
}

TEST_F(JacobianExecutorTest, execute_JacobianCommandFinished_DoesNotThrow)
{
  JacobianVelocityExecutor executor(mEENode);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto future = executor.execute(mSE3Command);

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

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent, 2E-2);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent, 2E-2);

  mSkeleton->getDof(0)->setPosition(-1.0);
  mSkeleton->getDof(1)->setPosition(-1.0);

  // Execute second traj.
  simulationClock = std::chrono::system_clock::now();
  beginning = simulationClock;
  future = executor.execute(mSE3Command);

  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();

  timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();

  EXPECT_NEAR(mSkeleton->getDof(0)->getPosition(), timeSpent - 1.0, 2E-2);
  EXPECT_NEAR(mSkeleton->getDof(1)->getPosition(), timeSpent - 1.0, 2E-2);
}
