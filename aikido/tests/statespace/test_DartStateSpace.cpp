#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/statespace/SE2StateSpace.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>
#include <aikido/statespace/MetaSkeletonStateSpace.hpp>
#include <gtest/gtest.h>
#include <dart/dynamics/dynamics.h>

using Eigen::Isometry3d;
using Eigen::Vector3d;
using dart::dynamics::Skeleton;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::PrismaticJoint;
using dart::dynamics::TranslationalJoint;
using dart::dynamics::FreeJoint;
using aikido::statespace::MetaSkeletonStateSpace;
using aikido::statespace::RealVectorStateSpace;
using aikido::statespace::SO2StateSpace;
using aikido::statespace::SE3StateSpace;

TEST(MetaSkeletonStateSpace, RevoluteJoint_WithoutBounds)
{
  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<RevoluteJoint>();

  MetaSkeletonStateSpace space(skeleton);
  ASSERT_EQ(1, space.getNumStates());

  auto state = space.createState();
  auto substate = state.getSubStateHandle<SO2StateSpace>(0);

  skeleton->setPosition(0, 5.);
  space.getStateFromMetaSkeleton(state);
  EXPECT_DOUBLE_EQ(5., substate.getAngle());

  substate.setAngle(6.);
  space.setStateOnMetaSkeleton(state);
  EXPECT_DOUBLE_EQ(6., skeleton->getPosition(0));
}

TEST(MetaSkeletonStateSpace, RevoluteJoint_WithBounds)
{
  auto skeleton = Skeleton::create();
  auto joint = skeleton->createJointAndBodyNodePair<RevoluteJoint>().first;
  joint->setPositionLowerLimit(0, -1.);

  MetaSkeletonStateSpace space(skeleton);
  ASSERT_EQ(1, space.getNumStates());

  auto subspace = space.getSubSpace<RealVectorStateSpace>(0);
  ASSERT_EQ(1, subspace->getDimension());

  auto state = space.createState();
  auto substate = state.getSubStateHandle<RealVectorStateSpace>(0);

  skeleton->setPosition(0, 5.);
  space.getStateFromMetaSkeleton(state);
  EXPECT_DOUBLE_EQ(5., substate.getValue()[0]);

  substate.getValue()[0] = 6.;
  space.setStateOnMetaSkeleton(state);
  EXPECT_DOUBLE_EQ(6., skeleton->getPosition(0));
}

TEST(MetaSkeletonStateSpace, PrismaticJoint)
{
  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<PrismaticJoint>();

  MetaSkeletonStateSpace space(skeleton);
  ASSERT_EQ(1, space.getNumStates());

  auto subspace = space.getSubSpace<RealVectorStateSpace>(0);
  ASSERT_EQ(1, subspace->getDimension());

  auto state = space.createState();
  auto substate = state.getSubStateHandle<RealVectorStateSpace>(0);

  skeleton->setPosition(0, 5.);
  space.getStateFromMetaSkeleton(state);
  EXPECT_DOUBLE_EQ(5., substate.getValue()[0]);

  substate.getValue()[0] = 6.;
  space.setStateOnMetaSkeleton(state);
  EXPECT_DOUBLE_EQ(6., skeleton->getPosition(0));
}

TEST(MetaSkeletonStateSpace, TranslationalJoint)
{
  Vector3d value1(1., 2., 3.);
  Vector3d value2(4., 5., 6.);

  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<TranslationalJoint>();

  MetaSkeletonStateSpace space(skeleton);
  ASSERT_EQ(1, space.getNumStates());

  auto subspace = space.getSubSpace<RealVectorStateSpace>(0);
  ASSERT_EQ(3, subspace->getDimension());

  auto state = space.createState();
  auto substate = state.getSubStateHandle<RealVectorStateSpace>(0);

  skeleton->setPositions(value1);
  space.getStateFromMetaSkeleton(state);
  EXPECT_TRUE(value1.isApprox(substate.getValue()));

  substate.setValue(value2);
  space.setStateOnMetaSkeleton(state);
  EXPECT_TRUE(value2.isApprox(skeleton->getPositions()));
}

TEST(MetaSkeletonStateSpace, FreeJoint)
{
  Isometry3d value1 = Isometry3d::Identity();
  value1.rotate(Eigen::AngleAxisd(M_PI_2, Vector3d::UnitZ()));
  value1.pretranslate(Vector3d(1., 2., 3.));

  Isometry3d value2 = Isometry3d::Identity();
  value2.rotate(Eigen::AngleAxisd(3 * M_PI_2, Vector3d::UnitZ()));
  value2.pretranslate(Vector3d(4., 5., 6.));

  auto skeleton = Skeleton::create();
  skeleton->createJointAndBodyNodePair<FreeJoint>();

  MetaSkeletonStateSpace space(skeleton);
  ASSERT_EQ(1, space.getNumStates());

  auto state = space.createState();
  auto substate = state.getSubStateHandle<SE3StateSpace>(0);

  skeleton->setPositions(FreeJoint::convertToPositions(value1));
  space.getStateFromMetaSkeleton(state);
  EXPECT_TRUE(value1.isApprox(substate.getIsometry()));

  substate.setIsometry(value2);
  space.setStateOnMetaSkeleton(state);
  EXPECT_TRUE(value2.isApprox(
    FreeJoint::convertToTransform(skeleton->getPositions())));
}

TEST(MetaSkeletonStateSpace, MultipleJoints)
{
  Vector3d value1(2., 3., 4.);
  Vector3d value2(6., 7., 8.);

  auto skeleton = Skeleton::create();
  auto joint1 = skeleton->createJointAndBodyNodePair<RevoluteJoint>().first;
  auto joint2 = skeleton->createJointAndBodyNodePair<TranslationalJoint>().first;

  MetaSkeletonStateSpace space(skeleton);
  ASSERT_EQ(2, space.getNumStates());

  auto state = space.createState();
  auto substate1 = state.getSubStateHandle<SO2StateSpace>(0);
  auto substate2 = state.getSubStateHandle<RealVectorStateSpace>(1);

  joint1->setPosition(0, 1.);
  joint2->setPositions(value1);
  space.getStateFromMetaSkeleton(state);
  EXPECT_EQ(1., substate1.getAngle());
  EXPECT_TRUE(value1.isApprox(value1));

  substate1.setAngle(5.);
  substate2.setValue(value2);
  space.setStateOnMetaSkeleton(state);
  EXPECT_EQ(5., substate1.getAngle());
  EXPECT_TRUE(value2.isApprox(substate2.getValue()));
}
