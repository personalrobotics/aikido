#include <gtest/gtest.h>
#include <aikido/control/ros/Conversions.hpp>
#include <dart/dart.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/statespace/Rn.hpp>
#include "eigen_tests.hpp"

using dart::dynamics::SkeletonPtr;
using dart::dynamics::Skeleton;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::BallJoint;
using aikido::trajectory::Spline;
using aikido::control::ros::convertTrajectoryToRosTrajectory;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::tests::make_vector;

static const double kTolerance{1e-6};

// TODO: We might want to merge this with test_Conversions.cpp
class ConvertTrajectoryToRosJointTrajectoryTests : public testing::Test
{
protected:
  void SetUp() override
  {
    // Create a single-DOF skeleton.
    auto skeleton = Skeleton::create();
    skeleton->createJointAndBodyNodePair<RevoluteJoint>();
    skeleton->setPosition(0, 0.);
    skeleton->getJoint(0)->setName("Joint1");

    mStateSpace = std::make_shared<MetaSkeletonStateSpace>(skeleton);
    auto startState = mStateSpace->createState();
    mStateSpace->getState(startState);

    // Spline trajectory
    mTrajectory = std::make_shared<Spline>(mStateSpace, 0.);
    mTrajectory->addSegment(make_vector(0.), 0.1, startState);

    // Timestep
    mTimestep = 0.1;

    // indexMap
    mIndexMap.insert(std::make_pair<size_t, size_t>(0, 0));

  }

  std::shared_ptr<Spline> mTrajectory;
  MetaSkeletonStateSpacePtr mStateSpace;

  double mTimestep;
  std::map<size_t, size_t> mIndexMap;
};

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, TrajectoryIsNull_Throws)
{
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(nullptr, mIndexMap, mTimestep);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, TimestepIsZero_Throws)
{
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory, mIndexMap, 0.0);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, TimestepIsNegative_Throws)
{
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory, mIndexMap, -0.1);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, InvalidIndexMap_Throws)
{
  std::map<size_t, size_t> indexMap;
  indexMap.insert(std::make_pair<size_t, size_t>(1,0));
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory, indexMap, 0.1);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, EmptyIndexMap_Throws)
{
  std::map<size_t, size_t> indexMap;
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(mTrajectory, indexMap, mTimestep);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, SkeletonHasUnsupportedJoint_Throws)
{
  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<BallJoint>();
  MetaSkeletonStateSpacePtr space = std::make_shared<MetaSkeletonStateSpace>(skeleton);

  auto trajectory = std::make_shared<Spline>(space, 0.0);
  EXPECT_THROW({
    convertTrajectoryToRosTrajectory(trajectory, mIndexMap, mTimestep);
  }, std::invalid_argument);
}

TEST_F(ConvertTrajectoryToRosJointTrajectoryTests, TrajectoryHasCorrectWaypoints)
{
  auto rosTrajectory = convertTrajectoryToRosTrajectory(mTrajectory, mIndexMap, mTimestep);
  EXPECT_EQ(2, rosTrajectory.points.size());
  for (int i = 0; i < 2; ++i)
  {
    ASSERT_DOUBLE_EQ(mTimestep*i, rosTrajectory.points[i].time_from_start.toSec());

    auto state = mStateSpace->createState();
    Eigen::VectorXd values;

    mTrajectory->evaluate(mTimestep*i, state);
    mStateSpace->convertStateToPositions(state, values);
    EXPECT_EIGEN_EQUAL(values, make_vector(rosTrajectory.points[i].positions[0]), kTolerance);
  }
  ASSERT_DOUBLE_EQ(mTimestep*(rosTrajectory.points.size()-1), mTrajectory->getEndTime());

  double timestep = 0.01;
  auto rosTrajectory2 = convertTrajectoryToRosTrajectory(mTrajectory, mIndexMap, timestep);
  EXPECT_EQ(11, rosTrajectory2.points.size());
  for (int i = 0; i < 11; ++i)
  {
    ASSERT_DOUBLE_EQ(timestep*i, rosTrajectory2.points[i].time_from_start.toSec());

    auto state = mStateSpace->createState();
    Eigen::VectorXd values;

    mTrajectory->evaluate(timestep*i, state);
    mStateSpace->convertStateToPositions(state, values);
    EXPECT_EIGEN_EQUAL(values, make_vector(rosTrajectory2.points[i].positions[0]), kTolerance);
  }
  ASSERT_DOUBLE_EQ(timestep*(rosTrajectory2.points.size()-1), mTrajectory->getEndTime());
}

