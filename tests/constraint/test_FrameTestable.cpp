#include <gtest/gtest.h>
#include <aikido/constraint/FrameTestable.hpp>
#include <aikido/statespace/SE3.hpp>
#include <aikido/statespace/SO2.hpp>
#include "MockConstraints.hpp"

using aikido::constraint::FrameTestable;
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
  EndEffectorTestable(std::shared_ptr<SE3> stateSpace) : mStateSpace(stateSpace)
  {
  }

  aikido::statespace::StateSpacePtr getStateSpace() const override
  {
    return mStateSpace;
  }

  bool isSatisfied(
      const aikido::statespace::StateSpace::State* _state,
      aikido::constraint::TestableOutcome* = nullptr /*_outcome*/)
      const override
  {
    auto st = static_cast<const SE3::State*>(_state);
    auto val = st->getIsometry();
    auto trans = val.translation();
    return (
        trans(0) > 0.5 && trans(0) < 1.5 && trans(1) > 0.0 && trans(1) < 1.0);
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
    auto robot = Skeleton::create("robot");

    // Root joint
    RevoluteJoint::Properties properties1;
    properties1.mAxis = Eigen::Vector3d::UnitZ();
    properties1.mName = "j1";
    auto bn1 = robot
                   ->createJointAndBodyNodePair<RevoluteJoint>(
                       nullptr, properties1, create_BodyNodeProperties("b1"))
                   .second;

    // Joint 2
    RevoluteJoint::Properties properties2;
    properties2.mAxis = Eigen::Vector3d::UnitZ();
    properties2.mName = "j2";
    properties2.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 1, 0);
    auto bn2 = robot
                   ->createJointAndBodyNodePair<RevoluteJoint>(
                       bn1, properties2, create_BodyNodeProperties("b2"))
                   .second;

    // End effector
    RevoluteJoint::Properties propertiesEE;
    propertiesEE.mAxis = Eigen::Vector3d::UnitZ();
    propertiesEE.mName = "ee";
    propertiesEE.mT_ParentBodyToJoint.translation() = Eigen::Vector3d(0, 1, 0);
    endEffector = robot
                      ->createJointAndBodyNodePair<RevoluteJoint>(
                          bn2, propertiesEE, create_BodyNodeProperties("b3"))
                      .second;

    // Statespace
    stateSpace = std::make_shared<MetaSkeletonStateSpace>(robot);
    auto se3 = std::make_shared<SE3>();
    poseConstraint = std::make_shared<EndEffectorTestable>(se3);
  }

  void setStateValue(
      const Eigen::Vector2d& value, MetaSkeletonStateSpace::State* state) const
  {
    auto j1Joint = stateSpace->getSubState<SO2::State>(state, 0);
    auto j2Joint = stateSpace->getSubState<SO2::State>(state, 1);
    j1Joint->setAngle(value[0]);
    j2Joint->setAngle(value[1]);
  }

  Eigen::Vector2d getStateValue(MetaSkeletonStateSpace::State* state) const
  {
    auto j1Joint = stateSpace->getSubState<SO2::State>(state, 0);
    auto j2Joint = stateSpace->getSubState<SO2::State>(state, 1);

    Eigen::Vector2d retVal(j1Joint->getAngle(), j2Joint->getAngle());
    return retVal;
  }

  BodyNodePtr endEffector;
  MetaSkeletonStateSpacePtr stateSpace;
  TestablePtr poseConstraint;
};

TEST_F(FrameTestableTest, ConstructorThrowsOnNullStateSpace)
{
  EXPECT_THROW(
      FrameTestable(nullptr, endEffector.get(), poseConstraint),
      std::invalid_argument);
}

TEST_F(FrameTestableTest, ConstructorThrowsOnNullFrame)
{
  EXPECT_THROW(
      FrameTestable(stateSpace, nullptr, poseConstraint),
      std::invalid_argument);
}

TEST_F(FrameTestableTest, ConstructorThrowsOnNullPoseConstraint)
{
  EXPECT_THROW(
      FrameTestable(stateSpace, endEffector.get(), nullptr),
      std::invalid_argument);
}

TEST_F(FrameTestableTest, ConstructorThrowsOnBadPoseConstraint)
{
  auto so2 = std::make_shared<aikido::statespace::SO2>();
  auto pconstraint = std::make_shared<PassingConstraint>(so2);

  EXPECT_THROW(
      FrameTestable(stateSpace, endEffector.get(), pconstraint),
      std::invalid_argument);
}

TEST_F(FrameTestableTest, StateSpaceMatch)
{
  FrameTestable fk(stateSpace, endEffector.get(), poseConstraint);
  EXPECT_EQ(stateSpace, fk.getStateSpace());
}

TEST_F(FrameTestableTest, SatifiedConstraint)
{
  Eigen::Vector2d pose(-M_PI * 0.25, -M_PI * 0.5);
  auto state = stateSpace->createState();
  setStateValue(pose, state);
  FrameTestable fk(stateSpace, endEffector.get(), poseConstraint);

  EXPECT_TRUE(fk.isSatisfied(state));
}

TEST_F(FrameTestableTest, UnsatisfiedConstraint)
{
  Eigen::Vector2d pose(0, 0);
  auto state = stateSpace->createState();
  setStateValue(pose, state);
  FrameTestable fk(stateSpace, endEffector.get(), poseConstraint);

  EXPECT_FALSE(fk.isSatisfied(state));
}
