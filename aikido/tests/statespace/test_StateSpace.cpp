#include <aikido/statespace/RealVectorStateSpace.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>
#include <aikido/statespace/SE2StateSpace.hpp>
#include <aikido/statespace/SE3StateSpace.hpp>
#include <aikido/statespace/CompoundStateSpace.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <dart/math/Geometry.h>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <typeinfo>       // std::bad_cast

using namespace aikido::statespace;
using namespace std;

TEST(RealVectorStateSpace, Compose)
{
  RealVectorStateSpace rvss(3);

  auto s1 = rvss.createState();
  s1.setValue(Eigen::Vector3d(1, 2, 3));

  auto s2 = rvss.createState();
  s2.setValue(Eigen::Vector3d(2, 3, 4));

  auto out = rvss.createState();
  rvss.compose(s1, s2, out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(3, 5, 7)));
}

TEST(RealVectorStateSpace, ExpMap)
{
  RealVectorStateSpace rvss(3);

  auto out = rvss.createState();
  rvss.expMap(Eigen::Vector3d(1, 2, 3), out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(1, 2, 3)));
}

TEST(SO2StateSpace, Compose)
{
  SO2StateSpace::State s1(M_PI/4);
  SO2StateSpace::State s2(M_PI/2);
  SO2StateSpace::State out;
  SO2StateSpace::State expected(3.0/4.0*M_PI);

  SO2StateSpace so2;
  so2.compose(&s1, &s2, &out);

  EXPECT_TRUE(out.getRotation().isApprox(expected.getRotation()));
}

TEST(SO2StateSpace, ExpMap)
{
  SO2StateSpace::State out;
  SO2StateSpace::State expected;

  SO2StateSpace so2;
  so2.expMap(Eigen::VectorXd(Eigen::VectorXd::Zero(1)), &out);

  EXPECT_TRUE(out.getRotation().isApprox(expected.getRotation()));
}

TEST(SO3StateSpace, Compose)
{
  SO3StateSpace::State identity;
  EXPECT_TRUE(identity.getQuaternion().isApprox(
    Eigen::Quaterniond::Identity()));

  SO3StateSpace::State s2(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3StateSpace::State s3(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3StateSpace::State expected(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));

  SO3StateSpace::State out;
  SO3StateSpace so3;
  so3.compose(&s2, &s3, &out);

  EXPECT_TRUE(expected.getQuaternion().isApprox(out.getQuaternion()));
}

TEST(SO3StateSpace, ExpMap)
{
  SO3StateSpace::State out;
  SO3StateSpace::State expected(Eigen::Quaterniond(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));

  SO3StateSpace so3;
  so3.expMap(Eigen::Vector3d(0, 0, M_PI_2), &out);

  EXPECT_TRUE(out.getQuaternion().isApprox(expected.getQuaternion()));
}

TEST(SE2StateSpace, Compose)
{
  SE2StateSpace space;

  auto identity = space.createState();
  EXPECT_TRUE(identity.getIsometry().isApprox(Eigen::Isometry2d::Identity()));

  Eigen::Isometry2d pose2 = Eigen::Isometry2d::Identity();
  pose2.rotate(Eigen::Rotation2Dd(M_PI_2));
  auto state2 = space.createState();
  state2.setIsometry(pose2);

  Eigen::Isometry2d pose3 = Eigen::Isometry2d::Identity();
  pose3.rotate(Eigen::Rotation2Dd(M_PI_4));
  auto state3 = space.createState();
  state3.setIsometry(pose3);

  Eigen::Isometry2d expected_pose = Eigen::Isometry2d::Identity();
  expected_pose.rotate(Eigen::Rotation2Dd(3. * M_PI_4));

  auto out = space.createState();
  space.compose(state2, state3, out);

  EXPECT_TRUE(expected_pose.isApprox(out.getIsometry()));
}

TEST(SE2StateSpace, ExpMap)
{
  SE2StateSpace::State out;

  Eigen::Isometry2d expected_pose = Eigen::Isometry2d::Identity();
  expected_pose.rotate(Eigen::Rotation2Dd(M_PI_2));

  SE2StateSpace se2;
  se2.expMap(Eigen::Vector3d(M_PI_2, 0, 0), &out);

  EXPECT_TRUE(out.getIsometry().isApprox(expected_pose));
}

TEST(SE3StateSpace, Compose)
{
  SE3StateSpace space;

  auto identity = space.createState();
  EXPECT_TRUE(identity.getIsometry().isApprox(Eigen::Isometry3d::Identity()));

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
  auto s2 = space.createState();
  s2.setIsometry(pose2);

  Eigen::Isometry3d pose3 = Eigen::Isometry3d::Identity();
  pose3.rotate(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX()));
  auto s3 = space.createState();
  s3.setIsometry(pose3);

  Eigen::Isometry3d expected = pose2 * pose3;

  SE3StateSpace::ScopedState out(&space);
  space.compose(s2, s3, out);

  EXPECT_TRUE(expected.isApprox(out.getIsometry()));
}

TEST(SE3StateSpace, ExpMap)
{
  SE3StateSpace::State out;

  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  expected_pose.translation() = Eigen::Vector3d(1, 2, 3);

  SE3StateSpace se2;
  Eigen::Vector6d twist(Eigen::Vector6d::Zero());
  twist.bottomRows(3) = Eigen::Vector3d(1, 2, 3);
  
  se2.expMap(twist, &out);

  EXPECT_TRUE(out.getIsometry().isApprox(expected_pose));
}

TEST(CompoundStateSpace, Compose)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({
    std::make_shared<SO2StateSpace>(),
    std::make_shared<RealVectorStateSpace>(2)
  });

  // TODO: This syntax is _really_ bad.
  CompoundStateSpace::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2StateSpace>(0).setAngle(M_PI_2);
  s1.getSubStateHandle<RealVectorStateSpace>(1).setValue(Vector2d(3., 4.));

  CompoundStateSpace::ScopedState s2 = space.createState();
  s2.getSubStateHandle<SO2StateSpace>(0).setAngle(M_PI_2);
  s2.getSubStateHandle<RealVectorStateSpace>(1).setValue(Vector2d(5., 10.));

  CompoundStateSpace::ScopedState out = space.createState();
  space.compose(s1, s2, out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(M_PI, out1);

  const Vector2d out2 = out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(8., 14.)));
}

TEST(CompoundStateSpace, ExpMap)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({
    std::make_shared<SO2StateSpace>(),
    std::make_shared<RealVectorStateSpace>(2)
  });

  CompoundStateSpace::ScopedState out = space.createState();

  space.expMap(Eigen::Vector3d(M_PI_2, 1, 2), out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(M_PI_2, out1);

  const Vector2d out2 = out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(1, 2)));
}
