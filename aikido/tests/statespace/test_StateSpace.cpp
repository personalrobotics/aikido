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

TEST(RealVectorStateSpace, EnforceBounds)
{
    Eigen::Matrix<double, 3, 2> bounds;
    bounds << -1, 1, -2, 2, -3, 3;

    RealVectorStateSpace rvss(bounds);
    auto state = rvss.createState();
    state.setValue(Eigen::Vector3d(-0.5, 0.0, 0.5));
    EXPECT_ANY_THROW(rvss.enforceBounds(state));

    // Ensure we can't set to bad values
    state.setValue(Eigen::Vector3d(2, 3, 4));
    EXPECT_TRUE(state.getValue().isApprox(Eigen::Vector3d(-0.5, 0., 0.5)));
}

TEST(RealVectorStateSpace, SatisfiesBounds)
{
    Eigen::Matrix<double, 3, 2> bounds;
    bounds << -1, 1, -2, 2, -3, 3;

    RealVectorStateSpace rvss(bounds);
    auto state = rvss.createState();
    state.setValue(Eigen::Vector3d(-0.5, 0.0, 0.5));
    EXPECT_TRUE(rvss.satisfiesBounds(state));
}

TEST(RealVectorStateSpace, CopyState)
{
    RealVectorStateSpace rvss(4);
    auto source = rvss.createState();
    auto dest = rvss.createState();
    source.setValue(Eigen::Vector4d(0, 1, 2, 3));
    rvss.copyState(dest, source);
    EXPECT_TRUE(dest.getValue().isApprox(source.getValue()));
}

TEST(RealVectorStateSpace, Distance)
{
    RealVectorStateSpace rvss(4);
    auto state1 = rvss.createState();
    auto state2 = rvss.createState();

    state1.setValue(Eigen::Vector4d(0, 1, 2, 3));
    state2.setValue(Eigen::Vector4d(-1, -2, -3, -4));
    EXPECT_EQ(rvss.distance(state1, state2), std::sqrt(84));
}

TEST(RealVectorStateSpace, EqualStates)
{
    RealVectorStateSpace rvss(4);
    auto state1 = rvss.createState();
    auto state2 = rvss.createState();

    state1.setValue(Eigen::Vector4d(0, 1, 2, 3));
    state2.setValue(Eigen::Vector4d(-1, -2, -3, -4));
    EXPECT_FALSE(rvss.equalStates(state1, state2));

    state2.setValue(state1.getValue());
    EXPECT_TRUE(rvss.equalStates(state1, state2));
}

TEST(RealVectorStateSpace, Interpolate)
{
    RealVectorStateSpace rvss(4);
    auto state1 = rvss.createState();
    auto state2 = rvss.createState();
    auto out = rvss.createState();

    state1.setValue(Eigen::Vector4d(0, 1, 2, 3));
    state2.setValue(Eigen::Vector4d(-1, -2, -3, -4));

    rvss.interpolate(state1, state2, 0, out);
    EXPECT_TRUE(out.getValue().isApprox(state1.getValue()));

    rvss.interpolate(state1, state2, 1, out);
    EXPECT_TRUE(out.getValue().isApprox(state2.getValue()));

    rvss.interpolate(state1, state2, 0.5, out);
    EXPECT_TRUE(out.getValue().isApprox(
                    Eigen::Vector4d(-0.5, -0.5, -0.5, -0.5)));

    EXPECT_ANY_THROW(rvss.interpolate(state1, state2, -0.5, out));
    EXPECT_ANY_THROW(rvss.interpolate(state1, state2, 1.1, out));
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

TEST(SO2StateSpace, EnforceBounds)
{
    SO2StateSpace so2;
    auto state = so2.createState();

    double expected = 5.3;
    state.setAngle(expected);
    so2.enforceBounds(state);
    EXPECT_NEAR(state.getAngle(), expected, 1e-6);
}

TEST(SO2StateSpace, SatisfiesBounds)
{
    SO2StateSpace so2;
    auto state = so2.createState();
    state.setAngle(-100.8);
    EXPECT_TRUE(so2.satisfiesBounds(state));
}

TEST(SO2StateSpace, CopyState)
{
    SO2StateSpace so2;
    auto dest = so2.createState();
    auto source = so2.createState();
    source.setAngle(3.14159);
    so2.copyState(dest, source);
    EXPECT_NEAR(dest.getAngle(), source.getAngle(), 1e-6);
}

TEST(SO2StateSpace, Distance)
{
    SO2StateSpace so2;
    auto state1 = so2.createState();
    auto state2 = so2.createState();
    state1.setAngle(1.3);
    state2.setAngle(1.5);
    EXPECT_NEAR(so2.distance(state1, state2), 0.2, 1e-6);

    state2.setAngle(6.0);
    EXPECT_NEAR(so2.distance(state1, state2), 
                2.*M_PI-state2.getAngle()+state1.getAngle(),
                1e-6);
}

TEST(SO2StateSpace, EqualStates)
{
    SO2StateSpace so2;
    auto state1 = so2.createState();
    auto state2 = so2.createState();
    
    state1.setAngle(0);
    state2.setAngle(4.0*M_PI);
    EXPECT_TRUE(so2.equalStates(state1, state2));

    state1.setAngle(0.3);
    state2.setAngle(0.3);
    EXPECT_TRUE(so2.equalStates(state1, state2));
}    

TEST(SO2StateSpace, Interpolate)
{
    SO2StateSpace so2;
    auto state1 = so2.createState();
    auto state2 = so2.createState();
    auto out = so2.createState();

    state1.setAngle(0.);
    state2.setAngle(2.*M_PI);
    so2.interpolate(state1, state2, 0, out);
    EXPECT_NEAR(out.getAngle(), state1.getAngle(), 1e-6);

    so2.interpolate(state1, state2, 1, out);
    EXPECT_NEAR(out.getAngle(), state2.getAngle(), 1e-6);

    so2.interpolate(state1, state2, 0.25, out);
    EXPECT_NEAR(out.getAngle(), M_PI_2, 1e-6);
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

TEST(SO3StateSpace, EnforceBounds)
{
    SO3StateSpace so3;
    auto state = so3.createState();

    auto quat = Eigen::Quaterniond(
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
    state.setQuaternion(quat);

    so3.enforceBounds(state);
    EXPECT_TRUE(state.getQuaternion().isApprox(quat));
}

TEST(SO3StateSpace, SatisfiesBounds)
{
    SO3StateSpace so3;
    auto state = so3.createState();

    // This shoudl always return true
    EXPECT_TRUE(so3.satisfiesBounds(state));
}

TEST(SO3StateSpace, CopyState)
{
    SO3StateSpace so3;
    auto source = so3.createState();
    auto dest = so3.createState();

    auto quat = Eigen::Quaterniond(
        Eigen::AngleAxisd(1.8*M_PI, Eigen::Vector3d::UnitX()));

    so3.copyState(dest, source);
    EXPECT_TRUE(source.getQuaternion().isApprox(dest.getQuaternion()));
}

TEST(SO3StateSpace, Distance)
{
    // TODO
}

TEST(SO3StateSpace, EqualStates)
{
    SO3StateSpace so3;
    auto state1 = so3.createState();
    auto state2 = so3.createState();

    auto q1 = Eigen::Quaterniond(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    auto q2 = Eigen::Quaterniond(
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
    auto q3 = Eigen::Quaterniond(
        Eigen::AngleAxisd(3.*M_PI, Eigen::Vector3d::UnitX()));

    state1.setQuaternion(q1);
    state2.setQuaternion(q2);
    EXPECT_FALSE(so3.equalStates(state1, state2));
    
    state2.setQuaternion(q1);
    EXPECT_TRUE(so3.equalStates(state1, state2));

    state2.setQuaternion(q3);
    EXPECT_TRUE(so3.equalStates(state1, state2));
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
