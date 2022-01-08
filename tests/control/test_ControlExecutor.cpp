#include <chrono>

#include <dart/dart.hpp>
#include <gtest/gtest.h>

#include <aikido/control/Executor.hpp>

using aikido::control::Executor;
using aikido::control::ExecutorType;
using ::dart::dynamics::BodyNode;
using ::dart::dynamics::BodyNodePtr;
using ::dart::dynamics::RevoluteJoint;
using ::dart::dynamics::Skeleton;
using ::dart::dynamics::SkeletonPtr;

const static std::chrono::milliseconds waitTime{0};

class ExecutorTest : public testing::Test
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

    auto bn1 = mSkeleton
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
  }

protected:
  ::dart::dynamics::SkeletonPtr mSkeleton;
};

TEST_F(ExecutorTest, constructor_BadTypes_Throws)
{
  EXPECT_THROW(
      Executor executor(
          std::set<ExecutorType>{ExecutorType::READONLY, ExecutorType::STATE},
          mSkeleton->getDofs()),
      std::invalid_argument);
}

TEST_F(ExecutorTest, constructor_Passes)
{
  EXPECT_NO_THROW(Executor executor(ExecutorType::STATE, mSkeleton->getDofs()));
}

TEST_F(ExecutorTest, constructor_LocksAndReleases)
{
  Executor executor1(ExecutorType::STATE, mSkeleton->getDofs());
  Executor executor2(ExecutorType::STATE, mSkeleton->getDofs());
  EXPECT_EQ(executor1.registerDofs(), true);
  EXPECT_EQ(executor2.registerDofs(), false);
  executor1.releaseDofs();
  executor2.registerDofs();
  EXPECT_EQ(executor2.registerDofs(), true);
  EXPECT_EQ(executor1.registerDofs(), false);
}

TEST_F(ExecutorTest, constructor_DifferentTypes)
{
  Executor executor1(ExecutorType::POSITION, mSkeleton->getDofs());
  Executor executor2(ExecutorType::VELOCITY, mSkeleton->getDofs());
  Executor executor3(
      std::set<ExecutorType>{ExecutorType::VELOCITY, ExecutorType::STATE},
      mSkeleton->getDofs());
  Executor executor4(ExecutorType::STATE, mSkeleton->getDofs());
  EXPECT_EQ(executor1.registerDofs(), true);
  EXPECT_EQ(executor2.registerDofs(), true);
  EXPECT_EQ(executor3.registerDofs(), false);
  EXPECT_EQ(executor4.registerDofs(), true);
  executor2.releaseDofs();
  EXPECT_EQ(executor3.registerDofs(), false);
  executor4.releaseDofs();
  EXPECT_EQ(executor3.registerDofs(), true);
}

TEST_F(ExecutorTest, constructor_DifferentDofs)
{
  std::vector<dart::dynamics::DegreeOfFreedom*> dof0{mSkeleton->getDof(0)};
  std::vector<dart::dynamics::DegreeOfFreedom*> dof1{mSkeleton->getDof(1)};
  Executor executor1(ExecutorType::STATE, dof0);
  Executor executor2(ExecutorType::STATE, dof1);
  EXPECT_EQ(executor1.registerDofs(), true);
  EXPECT_EQ(executor2.registerDofs(), true);
}
