#include <chrono>

#include <dart/dart.hpp>
#include <gtest/gtest.h>

#include <aikido/control/JointCommandExecutor.hpp>
#include <aikido/control/TrajectoryExecutor.hpp>
#include <aikido/robot/Robot.hpp>

using aikido::control::ExecutorType;
using aikido::robot::Robot;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::PrismaticJoint;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;

const static std::chrono::milliseconds waitTime{0};
const static std::chrono::milliseconds stepTime{100};

class RobotTest : public testing::Test
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

    mSkeleton->createJointAndBodyNodePair<PrismaticJoint>(
        bn1, jointProperties2, bodyProperties2);
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
};

TEST_F(RobotTest, constructor_Passes)
{
  EXPECT_NO_THROW(Robot robot(mSkeleton));
}

TEST_F(RobotTest, constructor_DefaultExecutors)
{
  Robot robot(mSkeleton);

  // Kinematic Trajectory Executor
  auto exec = robot.getActiveExecutor();
  EXPECT_NE(exec, nullptr);
  EXPECT_TRUE(exec->registerDofs());
  aikido::control::TrajectoryExecutorPtr trajExec;
  trajExec
      = std::dynamic_pointer_cast<aikido::control::TrajectoryExecutor>(exec);
  EXPECT_NE(trajExec, nullptr);

  // Kinematic Position Executor
  EXPECT_TRUE(robot.activateExecutor(ExecutorType::POSITION));
  EXPECT_FALSE(trajExec->registerDofs());
  exec = robot.getActiveExecutor();
  EXPECT_NE(exec, nullptr);
  EXPECT_TRUE(exec->registerDofs());
  std::shared_ptr<aikido::control::JointCommandExecutor<ExecutorType::POSITION>>
      posExec;
  posExec = std::dynamic_pointer_cast<
      aikido::control::JointCommandExecutor<ExecutorType::POSITION>>(exec);
  EXPECT_NE(posExec, nullptr);

  // Kinematic Velocity Executor
  EXPECT_TRUE(robot.activateExecutor(ExecutorType::VELOCITY));
  EXPECT_FALSE(trajExec->registerDofs());
  EXPECT_FALSE(posExec->registerDofs());
  exec = robot.getActiveExecutor();
  EXPECT_NE(exec, nullptr);
  EXPECT_TRUE(exec->registerDofs());
  std::shared_ptr<aikido::control::JointCommandExecutor<ExecutorType::VELOCITY>>
      velExec;
  velExec = std::dynamic_pointer_cast<
      aikido::control::JointCommandExecutor<ExecutorType::VELOCITY>>(exec);
  EXPECT_NE(velExec, nullptr);

  // Effort Executor should fail
  EXPECT_FALSE(robot.activateExecutor(ExecutorType::EFFORT));
  // But it should still deactivate active executor
  EXPECT_EQ(robot.getActiveExecutor(), nullptr);

  // Test activateExecutor by ID
  EXPECT_TRUE(robot.activateExecutor(0)); // Trajectory
  EXPECT_FALSE(posExec->registerDofs());
  EXPECT_TRUE(trajExec->registerDofs());
  EXPECT_FALSE(robot.activateExecutor(-1));
  EXPECT_FALSE(robot.activateExecutor(3));
  EXPECT_EQ(robot.getActiveExecutor(), nullptr);
  EXPECT_NO_THROW(robot.deactivateExecutor());

  // Text Explicit De-activation
  EXPECT_TRUE(robot.activateExecutor(0));
  EXPECT_EQ(robot.getActiveExecutor(), trajExec);
  robot.deactivateExecutor();
  EXPECT_EQ(robot.getActiveExecutor(), nullptr);

  // Clear Executors
  robot.clearExecutors();
  EXPECT_FALSE(robot.activateExecutor(0));
  EXPECT_EQ(robot.getActiveExecutor(), nullptr);

  // Re-Add Position and Velocity Executors (inactive)
  EXPECT_EQ(robot.registerExecutor(nullptr), -1);
  EXPECT_EQ(robot.registerExecutor(posExec), 0);
  EXPECT_EQ(robot.registerExecutor(velExec), 1);
  EXPECT_EQ(robot.getActiveExecutor(), nullptr);

  // Execute Position Command / Test step()
  std::vector<double> command(2, 1.0);
  std::chrono::duration<double> timeout(1.0);
  // No active executor
  auto future
      = robot.executeJointCommand<ExecutorType::POSITION>(command, timeout);
  EXPECT_THROW(future.get(), std::runtime_error);
  // Executor Wrong Type (1 = Velocity Executor)
  EXPECT_TRUE(robot.activateExecutor(1));
  future = robot.executeJointCommand<ExecutorType::POSITION>(command, timeout);
  EXPECT_THROW(future.get(), std::runtime_error);
  // Proper executor execution
  EXPECT_TRUE(robot.activateExecutor(ExecutorType::POSITION));
  future = robot.executeJointCommand<ExecutorType::POSITION>(command, timeout);
  std::future_status status;
  auto simulationClock = std::chrono::system_clock::now();
  do
  {
    simulationClock += stepTime;
    robot.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  future.get();
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 1.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 1.0);

  // Trajectory execution should fail (no active trajectoryexecutor)
  robot.deactivateExecutor();
  auto trajFuture = robot.executeTrajectory(nullptr);
  EXPECT_THROW(trajFuture.get(), std::runtime_error);
  robot.activateExecutor(0);
  trajFuture = robot.executeTrajectory(nullptr);
  EXPECT_THROW(trajFuture.get(), std::runtime_error);
}
