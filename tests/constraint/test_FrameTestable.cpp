#include <gtest/gtest.h>
#include <aikido/constraint/dart/FrameTestable.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/SO2.hpp>
#include "MockConstraints.hpp"

using aikido::constraint::DefaultTestableOutcome;
using aikido::constraint::dart::FrameTestable;
using aikido::constraint::Testable;
using aikido::constraint::TestablePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::SO2;
using aikido::statespace::SE3;
using dart::dynamics::Skeleton;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::BodyNode;

class EndEffectorTestable : public aikido::constraint::Testable
{
public:
  explicit EndEffectorTestable(std::shared_ptr<SE3> mStateSpace)
    : mStateSpace(mStateSpace)
  {
  }

  aikido::statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* _state,
      aikido::constraint::TestableOutcome* outcome = nullptr) const override
  {
    auto defaultOutcomeObject
        = aikido::constraint::dynamic_cast_or_throw<DefaultTestableOutcome>(
            outcome);

    auto st = static_cast<const SE3::State*>(_state);
    auto val = st->getIsometry();
    auto trans = val.translation();

    bool isSatisfiedResult
        = trans(0) > 0.5 && trans(0) < 1.5 && trans(1) > 0.0 && trans(1) < 1.0;
    if (defaultOutcomeObject)
      defaultOutcomeObject->setSatisfiedFlag(isSatisfiedResult);
    return isSatisfiedResult;
  }

  std::unique_ptr<TestableOutcome> createOutcome() const override
  {
    return std::unique_ptr<TestableOutcome>(new DefaultTestableOutcome);
  }

private:
  std::shared_ptr<SE3> mStateSpace;
};

static BodyNode::Properties create_BodyNodeProperties(const std::string& _name)
{
  BodyNode::Properties properties;
  properties.mName = _name;
  return properties;
}

class FrameTestableTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    mSkeleton = Skeleton::create("mSkeleton");

    // Root joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitZ();
    properties1.mName = "j1";
    auto bn1 = mSkeleton
                   ->createJointAndBodyNodePair<RevoluteJoint>(
                       nullptr, properties1, create_BodyNodeProperties("b1"))
                   .second;

    // Joint 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitZ();
    properties2.mName = "j2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 1, 0);
    auto bn2 = mSkeleton
                   ->createJointAndBodyNodePair<RevoluteJoint>(
                       bn1, properties2, create_BodyNodeProperties("b2"))
                   .second;

    // End-effector
    RevoluteJoint::Properties propertiesEE;
    propertiesEE.mAxis = Eigen::Vector3d::UnitZ();
    propertiesEE.mName = "ee";
    propertiesEE.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 1, 0);
    endEffector = mSkeleton
                      ->createJointAndBodyNodePair<RevoluteJoint>(
                          bn2, propertiesEE, create_BodyNodeProperties("b3"))
                      .second;

    // Statespace
    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton.get());
    auto se3 = std::make_shared<SE3>();
    poseConstraint = std::make_shared<EndEffectorTestable>(se3);
  }

  void setStateValue(
      const Eigen::Vector2d& value, MetaSkeletonStateSpace::State* state) const
  {
    auto j1Joint = mStateSpace->getSubState<SO2::State>(state, 0);
    auto j2Joint = mStateSpace->getSubState<SO2::State>(state, 1);
    j1Joint->setAngle(value[0]);
    j2Joint->setAngle(value[1]);
  }

  Eigen::Vector2d getStateValue(MetaSkeletonStateSpace::State* state) const
  {
    auto j1Joint = mStateSpace->getSubState<SO2::State>(state, 0);
    auto j2Joint = mStateSpace->getSubState<SO2::State>(state, 1);

    Eigen::Vector2d retVal(j1Joint->getAngle(), j2Joint->getAngle());
    return retVal;
  }

  BodyNodePtr endEffector;
  MetaSkeletonStateSpacePtr mStateSpace;
  dart::dynamics::SkeletonPtr mSkeleton;
  TestablePtr poseConstraint;
};

TEST_F(FrameTestableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(
      FrameTestable(nullptr, nullptr, endEffector.get(), poseConstraint),
      std::invalid_argument);
}

TEST_F(FrameTestableTest, ConstructorThrowsOnNullFrame)
{
  EXPECT_THROW(
      FrameTestable(mStateSpace, mSkeleton, nullptr, poseConstraint),
      std::invalid_argument);
}

TEST_F(FrameTestableTest, ConstructorThrowsOnNullPoseConstraint)
{
  EXPECT_THROW(
      FrameTestable(mStateSpace, mSkeleton, endEffector.get(), nullptr),
      std::invalid_argument);
}

TEST_F(FrameTestableTest, ConstructorThrowsOnBadPoseConstraint)
{
  auto so2 = std::make_shared<aikido::statespace::SO2>();
  auto pconstraint = std::make_shared<PassingConstraint>(so2);

  EXPECT_THROW(
      FrameTestable(mStateSpace, mSkeleton, endEffector.get(), pconstraint),
      std::invalid_argument);
}

TEST_F(FrameTestableTest, StateSpaceMatch)
{
  FrameTestable fk(mStateSpace, mSkeleton, endEffector.get(), poseConstraint);
  EXPECT_EQ(mStateSpace, fk.getStateSpace());
}

TEST_F(FrameTestableTest, SatifiedConstraint)
{
  Eigen::Vector2d pose(-M_PI * 0.25, -M_PI * 0.5);
  auto state = mStateSpace->createState();
  setStateValue(pose, state);
  FrameTestable fk(mStateSpace, mSkeleton, endEffector.get(), poseConstraint);

  EXPECT_TRUE(fk.isSatisfied(state));
}

TEST_F(FrameTestableTest, UnsatisfiedConstraint)
{
  Eigen::Vector2d pose(0, 0);
  auto state = mStateSpace->createState();
  setStateValue(pose, state);
  FrameTestable fk(mStateSpace, mSkeleton, endEffector.get(), poseConstraint);

  EXPECT_FALSE(fk.isSatisfied(state));
}
