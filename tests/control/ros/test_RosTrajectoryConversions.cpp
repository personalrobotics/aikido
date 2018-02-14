#include <dart/dart.hpp>
#include <gtest/gtest.h>
#include <aikido/control/ros/Conversions.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/trajectory/Spline.hpp>
#include "eigen_tests.hpp"

using dart::dynamics::SkeletonPtr;
using dart::dynamics::Skeleton;
using dart::dynamics::BodyNode;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::BallJoint;
using aikido::trajectory::Spline;
using aikido::control::ros::toRosJointTrajectory;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::tests::make_vector;

static const double kTolerance{1e-6};

// TODO: We might want to merge this with test_Conversions.cpp
class ToRosJointTrajectoryTests : public testing::Test
{
protected:
  void SetUp() override
  {
    // Create a single-DOF skeleton.
    auto skeleton = Skeleton::create();
    skeleton->createJointAndBodyNodePair<RevoluteJoint>();
    skeleton->getJoint(0)->setName("Joint1");

    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(skeleton.get());
    auto startState = mStateSpace->createState();
    mStateSpace->getState(skeleton.get(), startState);

    // Spline trajectory
    mTrajectory = std::make_shared<Spline>(mStateSpace, 0.);
    Eigen::Matrix<double, 1, 2> coeffs;
    coeffs << 0., 1.;
    mTrajectory->addSegment(coeffs, 0.1, startState);

    // Timestep
    mTimestep = 0.1;

    // Create a 2-DOF skeleton.
    auto skeleton2 = Skeleton::create();
    RevoluteJoint::Properties jointProperties;
    jointProperties.mName = "Joint1";
    BodyNode::Properties bnProperties;
    bnProperties.mName = "BodyNode1";

    auto bn = skeleton2
                  ->createJointAndBodyNodePair<RevoluteJoint, BodyNode>(
                      nullptr, jointProperties, bnProperties)
                  .second;
    skeleton2->createJointAndBodyNodePair<RevoluteJoint>(bn);
    skeleton2->getJoint(0)->setName("Joint1");
    skeleton2->getJoint(1)->setName("Joint2");
    skeleton2->getJoint(1)->setPosition(0, 1);

    mStateSpace2DOF = std::make_shared<MetaSkeletonStateSpace>(skeleton2.get());
    auto startState2DOF = mStateSpace2DOF->createState();
    mStateSpace2DOF->getState(skeleton2.get(), startState2DOF);

    // Spline trajectory
    mTrajectory2DOF = std::make_shared<Spline>(mStateSpace2DOF, 0.);
    Eigen::Matrix2d coeffs2;
    coeffs2 << 3., 1., 1., 1.;
    mTrajectory2DOF->addSegment(coeffs2, 0.1, startState2DOF);
  }

  std::shared_ptr<Spline> mTrajectory;
  MetaSkeletonStateSpacePtr mStateSpace;

  std::shared_ptr<Spline> mTrajectory2DOF;
  MetaSkeletonStateSpacePtr mStateSpace2DOF;

  double mTimestep;
};

TEST_F(ToRosJointTrajectoryTests, TrajectoryIsNull_Throws)
{
  EXPECT_THROW(
      { toRosJointTrajectory(nullptr, mTimestep); }, std::invalid_argument);
}

TEST_F(ToRosJointTrajectoryTests, TimestepIsZero_Throws)
{
  EXPECT_THROW(
      { toRosJointTrajectory(mTrajectory, 0.0); }, std::invalid_argument);
}

TEST_F(ToRosJointTrajectoryTests, TimestepIsNegative_Throws)
{
  EXPECT_THROW(
      { toRosJointTrajectory(mTrajectory, -0.1); }, std::invalid_argument);
}

TEST_F(ToRosJointTrajectoryTests, SkeletonHasUnsupportedJoint_Throws)
{
  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<BallJoint>();
  auto space = std::make_shared<MetaSkeletonStateSpace>(skeleton.get());

  auto trajectory = std::make_shared<Spline>(space, 0.0);
  EXPECT_THROW(
      { toRosJointTrajectory(trajectory, mTimestep); }, std::invalid_argument);
}

TEST_F(ToRosJointTrajectoryTests, TrajectoryHasCorrectWaypoints)
{
  auto rosTrajectory = toRosJointTrajectory(mTrajectory, mTimestep);

  EXPECT_EQ(2, rosTrajectory.points.size());
  for (int i = 0; i < 2; ++i)
  {
    ASSERT_DOUBLE_EQ(
        mTimestep * i, rosTrajectory.points[i].time_from_start.toSec());

    auto state = mStateSpace->createState();
    Eigen::VectorXd values;

    mTrajectory->evaluate(mTimestep * i, state);
    mStateSpace->convertStateToPositions(state, values);

    EXPECT_EIGEN_EQUAL(
        values, make_vector(rosTrajectory.points[i].positions[0]), kTolerance);
  }
  ASSERT_DOUBLE_EQ(
      mTimestep * (rosTrajectory.points.size() - 1), mTrajectory->getEndTime());

  // Finer timesteps
  double timestep = 0.01;
  auto rosTrajectory2 = toRosJointTrajectory(mTrajectory, timestep);

  EXPECT_EQ(11, rosTrajectory2.points.size());
  for (int i = 0; i < 11; ++i)
  {
    ASSERT_DOUBLE_EQ(
        timestep * i, rosTrajectory2.points[i].time_from_start.toSec());

    auto state = mStateSpace->createState();
    Eigen::VectorXd values;

    mTrajectory->evaluate(timestep * i, state);
    mStateSpace->convertStateToPositions(state, values);
    EXPECT_EIGEN_EQUAL(
        values, make_vector(rosTrajectory2.points[i].positions[0]), kTolerance);
  }
  ASSERT_DOUBLE_EQ(
      timestep * (rosTrajectory2.points.size() - 1), mTrajectory->getEndTime());
}
