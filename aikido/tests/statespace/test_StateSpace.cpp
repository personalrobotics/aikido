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

  RealVectorStateSpace::ScopedState s1 = rvss.createState();
  s1.getValue() = Eigen::Vector3d(1, 2, 3);

  RealVectorStateSpace::ScopedState s2 = rvss.createState();
  s2.getValue() = Eigen::Vector3d(2, 3, 4);

  RealVectorStateSpace::ScopedState out = rvss.createState();
  rvss.compose(*s1, *s2, *out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(3, 5, 7)));

}


TEST(SO2StateSpace, Compose)
{
  SO2StateSpace::State s1(M_PI/4);
  SO2StateSpace::State s2(M_PI/2);
  SO2StateSpace::State out;
  SO2StateSpace::State expected(3.0/4.0*M_PI);

  SO2StateSpace so2;
  so2.compose(s1, s2, out);

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
  so3.compose(s2, s3, out);

  EXPECT_TRUE(expected.getQuaternion().isApprox(out.getQuaternion()));
}

#if 0
TEST(SE2StateSpace, Compose)
{
  SE2StateSpace::State identity;
  EXPECT_TRUE(identity.getIsometry().isApprox(Eigen::Isometry2d::Identity()));

  Eigen::Isometry2d pose2 = Eigen::Isometry2d::Identity();
  pose2.rotate(Eigen::Rotation2Dd(M_PI_2));

  Eigen::Isometry2d pose3 = Eigen::Isometry2d::Identity();
  pose3.rotate(Eigen::Rotation2Dd(M_PI_4));

  Eigen::Isometry2d expected_pose = Eigen::Isometry2d::Identity();
  expected_pose.rotate(Eigen::Rotation2Dd(3. * M_PI_4));

  SE2StateSpace::State s2(pose2);
  SE2StateSpace::State s3(pose3);
  SE2StateSpace::State out;
  SE2StateSpace se2;
  se2.compose(s2, s3, out);

  EXPECT_TRUE(expected_pose.isApprox(out.getIsometry()));
}

TEST(SE3StateSpace, Compose)
{
  SE3StateSpace::State identity;
  EXPECT_TRUE(identity.getIsometry().isApprox(Eigen::Isometry3d::Identity()));

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d pose3 = Eigen::Isometry3d::Identity();
  pose3.rotate(Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitX()));

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.rotate(Eigen::AngleAxisd(3. * M_PI_4, Eigen::Vector3d::UnitX()));

  SE3StateSpace::State s2(pose2);
  SE3StateSpace::State s3(pose3);
  SE3StateSpace::State out;
  SE3StateSpace se3;
  se3.compose(s2, s3, out);

  EXPECT_TRUE(expected.isApprox(out.getIsometry()));
}
#endif


TEST(CompoundStateSpace, Compose)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({
    std::make_shared<SO2StateSpace>(),
    std::make_shared<RealVectorStateSpace>(2)
  });

  // TODO: This syntax is _really_ bad.
  CompoundStateSpace::ScopedState s1 = space.createState();
  space.getSubState<SO2StateSpace>(*s1, 0).setAngle(M_PI_2);

  space.getSubSpace<RealVectorStateSpace>(1).getValue(
    space.getSubState<RealVectorStateSpace>(*s1, 1)) = Vector2d(3., 4.);

  CompoundStateSpace::ScopedState s2 = space.createState();
  space.getSubState<SO2StateSpace>(*s2, 0).setAngle(M_PI_2);
  space.getSubSpace<RealVectorStateSpace>(1).getValue(
    space.getSubState<RealVectorStateSpace>(*s2, 1)) = Vector2d(5., 10.);

  CompoundStateSpace::ScopedState out = space.createState();
  space.compose(*s1, *s2, *out);

  const double out1 = space.getSubState<SO2StateSpace>(*out, 0).getAngle();
  const Vector2d out2 = space.getSubSpace<RealVectorStateSpace>(1).getValue(
    space.getSubState<RealVectorStateSpace>(*out, 1));
  EXPECT_DOUBLE_EQ(M_PI, out1);
  EXPECT_TRUE(out2.isApprox(Vector2d(8., 14.)));
}
