#include <dart/dart.hpp>
#include <gtest/gtest.h>

#include <aikido/control/ros/Conversions.hpp>

#include "eigen_tests.hpp"

using aikido::control::ros::toSplineJointTrajectory;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::tests::make_vector;
using dart::dynamics::SkeletonPtr;

static const double kTolerance{1e-6};

class ConvertJointTrajectoryTests : public testing::Test
{
protected:
  void SetUp() override
  {
    using dart::dynamics::BodyNode;
    using dart::dynamics::RevoluteJoint;

    // Create a single-DOF skeleton.
    mSkeleton = dart::dynamics::Skeleton::create("OneDofSkeleton");

    RevoluteJoint::Properties jointProperties;
    jointProperties.mName = "Joint1";

    BodyNode::Properties bnProperties;
    bnProperties.mName = "BodyNode1";

    mSkeleton->createJointAndBodyNodePair<RevoluteJoint, BodyNode>(
        nullptr, jointProperties, bnProperties);
    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton.get());

    // Create a two-waypoint trajectory for mSkeleton.
    mTwoWaypointMessage = trajectory_msgs::JointTrajectory{};
    mTwoWaypointMessage.joint_names.emplace_back(jointProperties.mName);
    mTwoWaypointMessage.points.resize(2);

    auto& waypoint1 = mTwoWaypointMessage.points[0];
    waypoint1.time_from_start = ros::Duration{0.};
    waypoint1.positions.assign({1.});

    auto& waypoint2 = mTwoWaypointMessage.points[1];
    waypoint2.time_from_start = ros::Duration{1.};
    waypoint2.positions.assign({2.});

    // 2-Joint skeleton
    mSkeleton2Joints = dart::dynamics::Skeleton::create("2JointSkeleton");

    RevoluteJoint::Properties jointProperties1;
    jointProperties1.mName = "Joint1";

    RevoluteJoint::Properties jointProperties2;
    jointProperties2.mName = "Joint2";

    BodyNode::Properties bnProperties1;
    bnProperties1.mName = "BodyNode1";

    BodyNode::Properties bnProperties2;
    bnProperties2.mName = "BodyNode2";

    auto bn1 = mSkeleton2Joints
                   ->createJointAndBodyNodePair<RevoluteJoint, BodyNode>(
                       nullptr, jointProperties1, bnProperties1)
                   .second;
    mSkeleton2Joints->createJointAndBodyNodePair<RevoluteJoint, BodyNode>(
        bn1, jointProperties2, bnProperties2);
    mStateSpace2Joints
        = std::make_shared<MetaSkeletonStateSpace>(mSkeleton2Joints.get());

    // Create a two-waypoint trajectory for mSkeleton2Joints,
    // with different ordering of joints.
    mTwoWaypointMessage2Joints = trajectory_msgs::JointTrajectory{};
    mTwoWaypointMessage2Joints.joint_names.emplace_back(jointProperties2.mName);
    mTwoWaypointMessage2Joints.joint_names.emplace_back(jointProperties1.mName);
    mTwoWaypointMessage2Joints.points.resize(2);

    auto& waypoint21 = mTwoWaypointMessage2Joints.points[0];
    waypoint21.time_from_start = ros::Duration{0.};
    waypoint21.positions.assign({1., 2.});

    auto& waypoint22 = mTwoWaypointMessage2Joints.points[1];
    waypoint22.time_from_start = ros::Duration{1.};
    waypoint22.positions.assign({2., 3.});
  }

  SkeletonPtr mSkeleton;
  std::shared_ptr<MetaSkeletonStateSpace> mStateSpace;
  trajectory_msgs::JointTrajectory mTwoWaypointMessage;

  SkeletonPtr mSkeleton2Joints;
  std::shared_ptr<MetaSkeletonStateSpace> mStateSpace2Joints;
  trajectory_msgs::JointTrajectory mTwoWaypointMessage2Joints;
};

TEST_F(ConvertJointTrajectoryTests, StateSpaceIsNull_Throws)
{
  EXPECT_THROW(
      { toSplineJointTrajectory(nullptr, mTwoWaypointMessage); },
      std::invalid_argument);
}

TEST_F(ConvertJointTrajectoryTests, NoWaypoints_Throws)
{
  auto zeroWaypointMessage = mTwoWaypointMessage;
  zeroWaypointMessage.points.clear();

  EXPECT_THROW(
      { toSplineJointTrajectory(mStateSpace, zeroWaypointMessage); },
      std::invalid_argument);
}

TEST_F(ConvertJointTrajectoryTests, LessThanTwoWaypoints_Throws)
{
  auto oneWaypointMessage = mTwoWaypointMessage;
  oneWaypointMessage.points.resize(1);

  EXPECT_THROW(
      { toSplineJointTrajectory(mStateSpace, oneWaypointMessage); },
      std::invalid_argument);
}

TEST_F(ConvertJointTrajectoryTests, IncorrectNumberOfJoints_Throws)
{
  mTwoWaypointMessage.joint_names.emplace_back("Joint1");

  EXPECT_THROW(
      { toSplineJointTrajectory(mStateSpace, mTwoWaypointMessage); },
      std::invalid_argument);
}

TEST_F(ConvertJointTrajectoryTests, TrajectoryHasUnknownJoint_Throws)
{
  mTwoWaypointMessage.joint_names[0] = "MissingJoint";

  EXPECT_THROW(
      { toSplineJointTrajectory(mStateSpace, mTwoWaypointMessage); },
      std::invalid_argument);
}

TEST_F(ConvertJointTrajectoryTests, StateSpaceHasUnknownJoint_Throws)
{
  mSkeleton->getJoint(0)->setName("MissingJoint");

  // This is necessary because the name change is not propagated to mStateSpace
  auto sspace = std::make_shared<MetaSkeletonStateSpace>(mSkeleton.get());

  EXPECT_THROW(
      { toSplineJointTrajectory(sspace, mTwoWaypointMessage); },
      std::invalid_argument);
}

TEST_F(ConvertJointTrajectoryTests, LinearTrajectory)
{
  const auto linearTwoWaypointMessage = mTwoWaypointMessage;
  const auto trajectory
      = toSplineJointTrajectory(mStateSpace, linearTwoWaypointMessage);

  ASSERT_TRUE(!!trajectory);
  ASSERT_DOUBLE_EQ(0., trajectory->getStartTime());
  ASSERT_DOUBLE_EQ(1., trajectory->getEndTime());
  EXPECT_DOUBLE_EQ(1., trajectory->getDuration());
  EXPECT_EQ(1, trajectory->getNumSegments());
  EXPECT_EQ(1, trajectory->getNumDerivatives());

  auto state = mStateSpace->createState();
  Eigen::VectorXd values;

  trajectory->evaluate(0., state);
  mStateSpace->convertStateToPositions(state, values);
  EXPECT_EIGEN_EQUAL(make_vector(1.), values, kTolerance);

  trajectory->evaluate(1., state);
  mStateSpace->convertStateToPositions(state, values);
  EXPECT_EIGEN_EQUAL(make_vector(2.), values, kTolerance);

  trajectory->evaluateDerivative(0.5, 1, values);
  EXPECT_EIGEN_EQUAL(make_vector(1.), values, kTolerance);

  trajectory->evaluateDerivative(0.5, 2, values);
  EXPECT_EIGEN_EQUAL(make_vector(0.), values, kTolerance);
}

TEST_F(ConvertJointTrajectoryTests, CubicTrajectory)
{
  auto cubicTwoWaypointMessage = mTwoWaypointMessage;
  cubicTwoWaypointMessage.points[0].velocities.assign({3.});
  cubicTwoWaypointMessage.points[1].velocities.assign({4.});

  const auto trajectory
      = toSplineJointTrajectory(mStateSpace, cubicTwoWaypointMessage);

  ASSERT_TRUE(!!trajectory);
  ASSERT_DOUBLE_EQ(0., trajectory->getStartTime());
  ASSERT_DOUBLE_EQ(1., trajectory->getEndTime());
  EXPECT_DOUBLE_EQ(1., trajectory->getDuration());
  EXPECT_EQ(1, trajectory->getNumSegments());
  EXPECT_EQ(3, trajectory->getNumDerivatives());

  auto state = mStateSpace->createState();
  Eigen::VectorXd values;

  trajectory->evaluate(0., state);
  mStateSpace->convertStateToPositions(state, values);
  EXPECT_EIGEN_EQUAL(make_vector(1.), values, kTolerance);

  trajectory->evaluate(1., state);
  mStateSpace->convertStateToPositions(state, values);
  EXPECT_EIGEN_EQUAL(make_vector(2.), values, kTolerance);

  trajectory->evaluateDerivative(0., 1, values);
  EXPECT_EIGEN_EQUAL(make_vector(3.), values, kTolerance);

  trajectory->evaluateDerivative(1., 1, values);
  EXPECT_EIGEN_EQUAL(make_vector(4.), values, kTolerance);
}

TEST_F(ConvertJointTrajectoryTests, QuinticTrajectory)
{
  auto quinticTwoWaypointMessage = mTwoWaypointMessage;
  quinticTwoWaypointMessage.points[0].velocities.assign({3.});
  quinticTwoWaypointMessage.points[0].accelerations.assign({5.});
  quinticTwoWaypointMessage.points[1].velocities.assign({4.});
  quinticTwoWaypointMessage.points[1].accelerations.assign({6.});

  const auto trajectory
      = toSplineJointTrajectory(mStateSpace, quinticTwoWaypointMessage);

  ASSERT_TRUE(!!trajectory);
  ASSERT_DOUBLE_EQ(0., trajectory->getStartTime());
  ASSERT_DOUBLE_EQ(1., trajectory->getEndTime());
  EXPECT_DOUBLE_EQ(1., trajectory->getDuration());
  EXPECT_EQ(1, trajectory->getNumSegments());
  EXPECT_EQ(5, trajectory->getNumDerivatives());

  auto state = mStateSpace->createState();
  Eigen::VectorXd values;

  trajectory->evaluate(0., state);
  mStateSpace->convertStateToPositions(state, values);
  EXPECT_EIGEN_EQUAL(make_vector(1.), values, kTolerance);

  trajectory->evaluate(1., state);
  mStateSpace->convertStateToPositions(state, values);
  EXPECT_EIGEN_EQUAL(make_vector(2.), values, kTolerance);

  trajectory->evaluateDerivative(0., 1, values);
  EXPECT_EIGEN_EQUAL(make_vector(3.), values, kTolerance);

  trajectory->evaluateDerivative(1., 1, values);
  EXPECT_EIGEN_EQUAL(make_vector(4.), values, kTolerance);

  trajectory->evaluateDerivative(0., 2, values);
  EXPECT_EIGEN_EQUAL(make_vector(5.), values, kTolerance);

  trajectory->evaluateDerivative(1., 2, values);
  EXPECT_EIGEN_EQUAL(make_vector(6.), values, kTolerance);
}

TEST_F(ConvertJointTrajectoryTests, DifferentOrderingOfJoints)
{
  const auto trajectory
      = toSplineJointTrajectory(mStateSpace2Joints, mTwoWaypointMessage2Joints);

  ASSERT_TRUE(!!trajectory);
  ASSERT_DOUBLE_EQ(0., trajectory->getStartTime());
  ASSERT_DOUBLE_EQ(1., trajectory->getEndTime());
  EXPECT_DOUBLE_EQ(1., trajectory->getDuration());
  EXPECT_EQ(1, trajectory->getNumSegments());
  EXPECT_EQ(1, trajectory->getNumDerivatives());

  auto state = mStateSpace2Joints->createState();
  Eigen::VectorXd values;

  trajectory->evaluate(0., state);
  mStateSpace2Joints->convertStateToPositions(state, values);
  EXPECT_EIGEN_EQUAL(make_vector(2., 1.), values, kTolerance);

  trajectory->evaluate(1., state);
  mStateSpace2Joints->convertStateToPositions(state, values);
  EXPECT_EIGEN_EQUAL(make_vector(3., 2.), values, kTolerance);

  trajectory->evaluateDerivative(0.5, 1, values);
  EXPECT_EIGEN_EQUAL(make_vector(1., 1.), values, kTolerance);

  trajectory->evaluateDerivative(0.5, 2, values);
  EXPECT_EIGEN_EQUAL(make_vector(0., 0.), values, kTolerance);
}

TEST_F(ConvertJointTrajectoryTests, StateSpaceMissingJoint_Throws)
{
  EXPECT_THROW(
      { toSplineJointTrajectory(mStateSpace, mTwoWaypointMessage2Joints); },
      std::invalid_argument);
}

TEST_F(
    ConvertJointTrajectoryTests,
    JointTrajectoryHasMultipleJointsWithSameName_Throws)
{
  auto trajectory = trajectory_msgs::JointTrajectory{};
  trajectory.joint_names.emplace_back("Joint1");
  trajectory.joint_names.emplace_back("Joint1");
  trajectory.points.resize(2);

  auto& waypoint1 = trajectory.points[0];
  waypoint1.time_from_start = ros::Duration{0.};
  waypoint1.positions.assign({1., 2.});

  auto& waypoint2 = trajectory.points[1];
  waypoint2.time_from_start = ros::Duration{1.};
  waypoint2.positions.assign({2., 3.});

  EXPECT_THROW(
      { toSplineJointTrajectory(mStateSpace2Joints, trajectory); },
      std::invalid_argument);
}

TEST_F(
    ConvertJointTrajectoryTests, StateSpaceHasMultipleJointsWithSameName_Throws)
{
  using dart::dynamics::BodyNode;
  using dart::dynamics::RevoluteJoint;

  // Create 2 skeletons with same joint names.
  auto skeleton1 = dart::dynamics::Skeleton::create();

  RevoluteJoint::Properties jointProperties;
  jointProperties.mName = "Joint1";

  skeleton1->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProperties);

  auto skeleton2 = dart::dynamics::Skeleton::create();
  skeleton2->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, jointProperties);

  auto groupSkeleton = dart::dynamics::Group::create();
  groupSkeleton->addJoint(skeleton1->getJoint(0));
  groupSkeleton->addJoint(skeleton2->getJoint(0));
  auto space = std::make_shared<MetaSkeletonStateSpace>(groupSkeleton.get());

  EXPECT_THROW(
      { toSplineJointTrajectory(space, mTwoWaypointMessage2Joints); },
      std::invalid_argument);
}
