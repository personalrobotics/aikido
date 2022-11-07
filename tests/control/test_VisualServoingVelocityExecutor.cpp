#include <chrono>

#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <gtest/gtest.h>

#include <aikido/control/VisualServoingVelocityExecutor.hpp>

using aikido::control::VisualServoingVelocityExecutor;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::PrismaticJoint;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;

const static std::chrono::milliseconds waitTime{0};
const static std::chrono::milliseconds stepTime{100};
const static std::chrono::duration<double> dStepTime{0.1};

class VisualServoingVelocityExecutorTest : public testing::Test
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

    mProperties = VisualServoingVelocityExecutor::Properties();
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
  ::dart::dynamics::BodyNode* mEENode;
  VisualServoingVelocityExecutor::Properties mProperties;
};

TEST_F(VisualServoingVelocityExecutorTest, constructor_NullBody_Throws)
{
  EXPECT_THROW(VisualServoingVelocityExecutor(nullptr), std::invalid_argument);
}

TEST_F(VisualServoingVelocityExecutorTest, constructor_ZeroVel_Throws)
{
  mProperties.mApproachVelocity = 0.0;
  EXPECT_THROW(
      VisualServoingVelocityExecutor(mEENode, nullptr, mProperties),
      std::invalid_argument);
}

TEST_F(VisualServoingVelocityExecutorTest, constructor_Passes)
{
  EXPECT_NO_THROW(VisualServoingVelocityExecutor executor(mEENode));
}

TEST_F(VisualServoingVelocityExecutorTest, execute_NullPerception_Throws)
{
  VisualServoingVelocityExecutor executor(mEENode);
  EXPECT_THROW(executor.execute(nullptr).get(), std::runtime_error);
}

TEST_F(VisualServoingVelocityExecutorTest, execute_PerceptionTimeout)
{
  VisualServoingVelocityExecutor executor(mEENode);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto future = executor.execute([] { return nullptr; });

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  EXPECT_EQ(future.get(), -1);

  double timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();

  EXPECT_NEAR(timeSpent, mProperties.mTimeout.count(), dStepTime.count());

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);
}

TEST_F(VisualServoingVelocityExecutorTest, execute_WaitOnFuture_NoMovement)
{
  VisualServoingVelocityExecutor executor(mEENode);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto goal
      = std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity());
  auto future = executor.execute([goal] { return goal; });

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  EXPECT_EQ(future.get(), 0);

  double timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();
  EXPECT_NEAR(timeSpent, dStepTime.count(), 1E-4);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);
}

TEST_F(
    VisualServoingVelocityExecutorTest, execute_WaitOnFuture_CommandWasExecuted)
{
  mProperties.mApproachVelocity = 0.1;
  VisualServoingVelocityExecutor executor(mEENode, nullptr, mProperties);

  EXPECT_DOUBLE_EQ(mSkeleton->getDof(0)->getPosition(), 0.0);
  EXPECT_DOUBLE_EQ(mSkeleton->getDof(1)->getPosition(), 0.0);

  auto simulationClock = std::chrono::system_clock::now();
  auto beginning = simulationClock;
  auto goal
      = std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity());
  goal->translation() << 1.0, 1.0, 0.0;
  auto future = executor.execute([goal] { return goal; });

  std::future_status status;
  do
  {
    simulationClock += stepTime;
    executor.step(simulationClock);
    status = future.wait_for(waitTime);
  } while (status != std::future_status::ready);

  EXPECT_EQ(future.get(), 0);

  double timeSpent
      = std::chrono::duration<double>(simulationClock - beginning).count();
  EXPECT_NEAR(
      timeSpent,
      goal->translation().norm() / mProperties.mApproachVelocity,
      dStepTime.count());

  EXPECT_NEAR(
      mSkeleton->getDof(0)->getPosition(),
      goal->translation()(0),
      mProperties.mGoalTolerance);
  EXPECT_NEAR(
      mSkeleton->getDof(1)->getPosition(),
      goal->translation()(1),
      mProperties.mGoalTolerance);
}
