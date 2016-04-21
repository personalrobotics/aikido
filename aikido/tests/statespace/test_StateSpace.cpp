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
#include <typeinfo>  // std::bad_cast

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

TEST(RealVectorStateSpace, Identity)
{
  RealVectorStateSpace rvss(3);

  auto s1 = rvss.createState();
  s1.setValue(Eigen::Vector3d(1, 2, 3));

  auto ident = rvss.createState();
  rvss.getIdentity(ident);

  auto out = rvss.createState();
  rvss.compose(s1, ident, out);

  EXPECT_TRUE(out.getValue().isApprox(s1.getValue()));
}

TEST(RealVectorStateSpace, Inverse)
{
  RealVectorStateSpace rvss(3);

  auto s1 = rvss.createState();
  s1.setValue(Eigen::Vector3d(1, 2, 3));
  auto ident = rvss.createState();
  rvss.getIdentity(ident);

  auto inv = rvss.createState();
  rvss.getInverse(s1, inv);

  auto out = rvss.createState();
  rvss.compose(s1, inv, out);

  EXPECT_TRUE(out.getValue().isApprox(ident.getValue()));
}

TEST(RealVectorStateSpace, ExpMap)
{
  RealVectorStateSpace rvss(3);

  auto out = rvss.createState();
  rvss.expMap(Eigen::Vector3d(1, 2, 3), out);

  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector3d(1, 2, 3)));
}

TEST(RealVectorStateSpace, LogMap)
{
  RealVectorStateSpace rvss(3);

  auto state = rvss.createState();
  rvss.setValue(state, Eigen::Vector3d(1, 2, 3));

  Eigen::VectorXd out;
  rvss.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(1, 2, 3)));

  rvss.expMap(Eigen::Vector3d(4, 5, 7), state);
  rvss.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(4, 5, 7)));
}

TEST(RealVectorStateSpace, CopyState)
{
  RealVectorStateSpace rvss(4);
  auto source = rvss.createState();
  auto dest = rvss.createState();
  source.setValue(Eigen::Vector4d(0, 1, 2, 3));
  rvss.copyState(source, dest);
  EXPECT_TRUE(dest.getValue().isApprox(source.getValue()));
}

TEST(SO2StateSpace, Compose)
{
  SO2StateSpace::State s1(M_PI / 4);
  SO2StateSpace::State s2(M_PI / 2);
  SO2StateSpace::State out;
  SO2StateSpace::State expected(3.0 / 4.0 * M_PI);

  SO2StateSpace so2;
  so2.compose(&s1, &s2, &out);

  EXPECT_TRUE(out.getRotation().isApprox(expected.getRotation()));
}

TEST(SO2StateSpace, Identity)
{
  SO2StateSpace so2;
  auto s1 = so2.createState();
  s1.setAngle(M_PI / 4);

  auto ident = so2.createState();
  so2.getIdentity(ident);

  auto out = so2.createState();
  so2.compose(s1, ident, out);
  EXPECT_DOUBLE_EQ(s1.getAngle(), out.getAngle());
}

TEST(SO2StateSpace, Inverse)
{
  SO2StateSpace so2;
  auto s1 = so2.createState();
  s1.setAngle(M_PI / 5);

  auto ident = so2.createState();
  so2.getIdentity(ident);

  auto inv = so2.createState();
  so2.getInverse(s1, inv);

  auto out = so2.createState();
  so2.compose(s1, inv, out);

  EXPECT_DOUBLE_EQ(ident.getAngle(), out.getAngle());
}

TEST(SO2StateSpace, ExpMap)
{
  SO2StateSpace::State out;
  SO2StateSpace::State expected;

  SO2StateSpace so2;
  so2.expMap(Eigen::VectorXd(Eigen::VectorXd::Zero(1)), &out);

  EXPECT_TRUE(out.getRotation().isApprox(expected.getRotation()));
}

TEST(SO2StateSpace, LogMap)
{
  SO2StateSpace::State state;
  SO2StateSpace so2;
  so2.setAngle(&state, M_PI / 5);

  Eigen::VectorXd out;
  so2.logMap(&state, out);
  EXPECT_DOUBLE_EQ(so2.getAngle(&state), out[0]);

  Eigen::VectorXd in(1);
  in[0] = M_PI / 4;
  so2.expMap(in, &state);
  so2.logMap(&state, out);
  EXPECT_TRUE(out.isApprox(in));
}

TEST(SO2StateSpace, CopyState)
{
  SO2StateSpace so2;
  auto dest = so2.createState();
  auto source = so2.createState();
  source.setAngle(3.14159);
  so2.copyState(source, dest);
  EXPECT_DOUBLE_EQ(source.getAngle(), dest.getAngle());
}

TEST(SO3StateSpace, Compose)
{
  SO3StateSpace::State identity;
  EXPECT_TRUE(
      identity.getQuaternion().isApprox(Eigen::Quaterniond::Identity()));

  SO3StateSpace::State s2(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3StateSpace::State s3(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3StateSpace::State expected(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));

  SO3StateSpace::State out;
  SO3StateSpace so3;
  so3.compose(&s2, &s3, &out);

  EXPECT_TRUE(expected.getQuaternion().isApprox(out.getQuaternion()));
}

TEST(SO3StateSpace, Identity)
{
  SO3StateSpace so3;
  SO3StateSpace::State s1(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3StateSpace::State ident;
  so3.getIdentity(&ident);

  SO3StateSpace::State out;
  so3.compose(&s1, &ident, &out);

  EXPECT_TRUE(s1.getQuaternion().isApprox(out.getQuaternion()));
}

TEST(SO3StateSpace, Inverse)
{
  SO3StateSpace so3;
  SO3StateSpace::State s1(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3StateSpace::State ident;
  so3.getIdentity(&ident);
  SO3StateSpace::State inv;
  so3.getInverse(&s1, &inv);

  SO3StateSpace::State out;
  so3.compose(&s1, &inv, &out);

  EXPECT_TRUE(ident.getQuaternion().isApprox(out.getQuaternion()));
}

TEST(SO3StateSpace, ExpMap)
{
  SO3StateSpace::State out;
  SO3StateSpace::State expected(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));

  SO3StateSpace so3;
  so3.expMap(Eigen::Vector3d(0, 0, M_PI_2), &out);

  EXPECT_TRUE(out.getQuaternion().isApprox(expected.getQuaternion()));
}

TEST(SO3StateSpace, LogMap)
{
  SO3StateSpace so3;
  auto state = so3.createState();
  so3.setQuaternion(state, Eigen::Quaterniond(Eigen::AngleAxisd(
                               M_PI_2, Eigen::Vector3d::UnitZ())));

  Eigen::VectorXd out;
  so3.logMap(state, out);
  EXPECT_TRUE(Eigen::Vector3d(0, 0, M_PI_2).isApprox(out));

  so3.expMap(Eigen::Vector3d(M_PI, M_PI_2, M_PI / 5), state);
  so3.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI, M_PI_2, M_PI / 5)));
}

TEST(SO3StateSpace, CopyState)
{
  SO3StateSpace so3;
  auto source = so3.createState();
  auto dest = so3.createState();

  auto quat = Eigen::Quaterniond(
      Eigen::AngleAxisd(1.8 * M_PI, Eigen::Vector3d::UnitX()));

  so3.copyState(source, dest);
  EXPECT_TRUE(source.getQuaternion().isApprox(dest.getQuaternion()));
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

TEST(SE2StateSpace, Identity)
{
  SE2StateSpace space;
  Eigen::Isometry2d pose1 = Eigen::Isometry2d::Identity();
  pose1.rotate(Eigen::Rotation2Dd(M_PI_2));
  auto state1 = space.createState();
  state1.setIsometry(pose1);

  auto ident = space.createState();
  space.getIdentity(ident);

  auto out = space.createState();
  space.compose(state1, ident, out);

  EXPECT_TRUE(state1.getIsometry().isApprox(out.getIsometry()));
}

TEST(SE2StateSpace, Inverse)
{
  SE2StateSpace space;
  Eigen::Isometry2d pose1 = Eigen::Isometry2d::Identity();
  pose1.rotate(Eigen::Rotation2Dd(M_PI_2));
  auto state1 = space.createState();
  state1.setIsometry(pose1);

  auto ident = space.createState();
  space.getIdentity(ident);

  auto inv = space.createState();
  space.getInverse(state1, inv);

  auto out = space.createState();
  space.compose(state1, inv, out);

  EXPECT_TRUE(ident.getIsometry().isApprox(out.getIsometry()));
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

TEST(SE2StateSpace, LogMap)
{
  SE2StateSpace se2;
  auto state = se2.createState();
  Eigen::Isometry2d pose1 = Eigen::Isometry2d::Identity();
  pose1.rotate(Eigen::Rotation2Dd(M_PI_2));
  pose1.translation() = Eigen::Vector2d(3, 4);
  se2.setIsometry(state, pose1);

  Eigen::VectorXd out;
  se2.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI_2, 3, 4)));

  se2.expMap(Eigen::Vector3d(M_PI / 6, 4, 6), state);
  se2.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI / 6, 4, 6)));
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

TEST(SE3StateSpace, Identity)
{
  SE3StateSpace space;

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
  auto s1 = space.createState();
  s1.setIsometry(pose1);

  auto ident = space.createState();
  space.getIdentity(ident);

  auto out = space.createState();
  space.compose(s1, ident, out);
  EXPECT_TRUE(s1.getIsometry().isApprox(out.getIsometry()));
}

TEST(SE3StateSpace, Inverse)
{
  SE3StateSpace space;

  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
  auto s1 = space.createState();
  s1.setIsometry(pose1);

  auto ident = space.createState();
  space.getIdentity(ident);

  auto inv = space.createState();
  space.getInverse(s1, inv);

  auto out = space.createState();
  space.compose(s1, inv, out);
  EXPECT_TRUE(ident.getIsometry().isApprox(out.getIsometry()));
}

TEST(SE3StateSpace, ExpMap)
{
  SE3StateSpace::State out;

  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  expected_pose.translation() = Eigen::Vector3d(1, 2, 3);

  SE3StateSpace se3;
  Eigen::Vector6d twist(Eigen::Vector6d::Zero());
  twist.bottomRows(3) = Eigen::Vector3d(1, 2, 3);

  se3.expMap(twist, &out);

  EXPECT_TRUE(out.getIsometry().isApprox(expected_pose));
}

TEST(SE3StateSpace, LogMap)
{
  SE3StateSpace se3;
  Eigen::Vector6d twist;
  twist << M_PI / 5, 0, 0, 1, 2, 3;

  auto state = se3.createState();
  Eigen::VectorXd out;
  se3.expMap(twist, state);
  se3.logMap(state, out);
  EXPECT_TRUE(out.isApprox(twist));
}

TEST(CompoundStateSpace, Compose)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

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

  const Vector2d out2 =
      out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(8., 14.)));
}

TEST(CompoundStateSpace, Identity)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2StateSpace>(0).setAngle(M_PI_2);
  s1.getSubStateHandle<RealVectorStateSpace>(1).setValue(Vector2d(3., 4.));

  CompoundStateSpace::ScopedState ident = space.createState();
  space.getIdentity(ident);

  CompoundStateSpace::ScopedState out = space.createState();
  space.compose(s1, ident, out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(M_PI_2, out1);

  const Vector2d out2 =
      out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(3., 4.)));
}

TEST(CompoundStateSpace, Inverse)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState s1 = space.createState();
  s1.getSubStateHandle<SO2StateSpace>(0).setAngle(M_PI_2);
  s1.getSubStateHandle<RealVectorStateSpace>(1).setValue(Vector2d(3., 4.));

  CompoundStateSpace::ScopedState ident = space.createState();
  space.getIdentity(ident);

  CompoundStateSpace::ScopedState inv = space.createState();
  space.getInverse(s1, inv);

  CompoundStateSpace::ScopedState out = space.createState();
  space.compose(s1, inv, out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  const double iout1 = ident.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(iout1, out1);

  const Vector2d out2 =
      out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  const Vector2d iout2 =
      ident.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(iout2));
}

TEST(CompoundStateSpace, ExpMap)
{
  using Eigen::Vector2d;

  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState out = space.createState();

  space.expMap(Eigen::Vector3d(M_PI_2, 1, 2), out);

  const double out1 = out.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(M_PI_2, out1);

  const Vector2d out2 =
      out.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(Vector2d(1, 2)));
}

TEST(CompoundStateSpace, LogMap)
{
  CompoundStateSpace space({std::make_shared<SO2StateSpace>(),
                            std::make_shared<RealVectorStateSpace>(2)});

  CompoundStateSpace::ScopedState state = space.createState();

  Eigen::VectorXd out;
  space.expMap(Eigen::Vector3d(M_PI_2, 1, 2), state);
  space.logMap(state, out);

  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI_2, 1, 2)));
}

TEST(CompoundStateSpace, CopyState)
{
  CompoundStateSpace space({
      std::make_shared<SO2StateSpace>(),
      std::make_shared<RealVectorStateSpace>(3),
      std::make_shared<SO3StateSpace>(),
  });

  auto source = space.createState();
  auto dest = space.createState();

  double angle = M_PI;
  auto rv = Eigen::Vector3d(3, 4, 5);
  auto quat =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

  source.getSubStateHandle<SO2StateSpace>(0).setAngle(angle);
  source.getSubStateHandle<RealVectorStateSpace>(1).setValue(rv);
  source.getSubStateHandle<SO3StateSpace>(2).setQuaternion(quat);

  space.copyState(source, dest);

  const double out1 = dest.getSubStateHandle<SO2StateSpace>(0).getAngle();
  EXPECT_DOUBLE_EQ(angle, out1);

  auto out2 = dest.getSubStateHandle<RealVectorStateSpace>(1).getValue();
  EXPECT_TRUE(out2.isApprox(rv));

  auto out3 = dest.getSubStateHandle<SO3StateSpace>(2).getQuaternion();
  EXPECT_TRUE(out3.isApprox(quat));
}
