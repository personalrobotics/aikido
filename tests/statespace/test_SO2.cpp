#include <gtest/gtest.h>
#include <aikido/statespace/SO2.hpp>
#include "eigen_tests.hpp"

using aikido::statespace::SO2;
using aikido::tests::make_vector;
using Eigen::Rotation2Dd;

static constexpr double TOLERANCE{1e-6};

TEST(SO2, BoundedAngle)
{
  SO2::State s1(3 * M_PI_2);
  EXPECT_DOUBLE_EQ(s1.toAngle(), -M_PI_2);
}

TEST(SO2, Compose)
{
  SO2::State s1(M_PI_4);
  SO2::State s2(M_PI_2);
  SO2::State out;
  SO2::State expected(3.0 * M_PI_4);

  SO2 so2;
  so2.compose(&s1, &s2, &out);

  EXPECT_TRUE(out.toRotation().isApprox(expected.toRotation()));
  EXPECT_DOUBLE_EQ(out.toAngle(), expected.toAngle());
}

TEST(SO2, ComposeWrapped)
{
  SO2::State s1(M_PI_4);
  SO2::State s2(3 * M_PI_2);
  SO2::State out;
  SO2::State expected(-M_PI_4);

  SO2 so2;
  so2.compose(&s1, &s2, &out);

  EXPECT_TRUE(out.toRotation().isApprox(expected.toRotation()));
  EXPECT_DOUBLE_EQ(out.toAngle(), expected.toAngle());
}

TEST(SO2, Identity)
{
  SO2 so2;
  auto s1 = so2.createState();
  s1.fromAngle(M_PI / 4);

  auto ident = so2.createState();
  so2.getIdentity(ident);

  auto out = so2.createState();
  so2.compose(s1, ident, out);
  EXPECT_DOUBLE_EQ(s1.toAngle(), out.toAngle());
}

TEST(SO2, Inverse)
{
  SO2 so2;
  auto s1 = so2.createState();
  s1.fromAngle(M_PI / 5);

  auto ident = so2.createState();
  so2.getIdentity(ident);

  auto inv = so2.createState();
  so2.getInverse(s1, inv);

  auto out = so2.createState();
  so2.compose(s1, inv, out);

  EXPECT_DOUBLE_EQ(ident.toAngle(), out.toAngle());
}

TEST(SO2, ExpMap)
{
  SO2::State out;
  SO2::State expected;

  SO2 so2;

  so2.expMap(make_vector(0.), &out);
  EXPECT_EIGEN_EQUAL(
      Rotation2Dd(0.).matrix(), expected.toRotation().matrix(), TOLERANCE);

  so2.expMap(make_vector(2 * M_PI), &out);
  EXPECT_EIGEN_EQUAL(
      Rotation2Dd(0.).matrix(), out.toRotation().matrix(), TOLERANCE);

  so2.expMap(make_vector(-3 * M_PI), &out);
  EXPECT_EIGEN_EQUAL(
      Rotation2Dd(M_PI).matrix(), out.toRotation().matrix(), TOLERANCE);
}

TEST(SO2, LogMap)
{
  SO2::State state;
  SO2 so2;
  so2.fromAngle(&state, M_PI / 5);

  Eigen::VectorXd out;
  so2.logMap(&state, out);
  EXPECT_DOUBLE_EQ(so2.toAngle(&state), out[0]);

  Eigen::VectorXd in(1);
  in[0] = M_PI / 4;
  so2.expMap(in, &state);
  so2.logMap(&state, out);
  EXPECT_TRUE(out.isApprox(in));
}

TEST(SO2, LogMapLargeAngles)
{
  SO2::State state;
  SO2 so2;
  so2.fromAngle(&state, 3 * M_PI_2);

  Eigen::VectorXd out;
  so2.logMap(&state, out);
  EXPECT_DOUBLE_EQ(so2.toAngle(&state), out[0]);

  Eigen::VectorXd in(1);
  in[0] = 11 * M_PI / 6;
  so2.expMap(in, &state);
  so2.logMap(&state, out);

  EXPECT_FALSE(out.isApprox(in));
  in[0] = -M_PI / 6;
  EXPECT_TRUE(out.isApprox(in));
}

TEST(SO2, CopyState)
{
  SO2 so2;
  auto dest = so2.createState();
  auto source = so2.createState();
  source.fromAngle(3.14159);
  so2.copyState(source, dest);
  EXPECT_DOUBLE_EQ(source.toAngle(), dest.toAngle());
}

TEST(SO2, PrintState)
{
  SO2 so2;
  auto source = so2.createState();
  source.fromAngle(M_PI);
  so2.print(source, std::cout);
}

TEST(SO2, GeodesicInterpolationWithinWrap)
{
  SO2::State from(0.0);
  SO2::State to(5 * M_PI / 3);
  SO2::State toMinusFrom;
  SO2::State fromInv;

  SO2 so2;
  so2.getInverse(&from, &fromInv);
  so2.compose(&fromInv, &to, &toMinusFrom);

  Eigen::VectorXd tangentVector;
  so2.logMap(&toMinusFrom, tangentVector);

  SO2::State relativeState;
  so2.expMap(0.5 * tangentVector, &relativeState);

  SO2::State halfPointState;
  so2.compose(&from, &relativeState, &halfPointState);

  SO2::State expected(-M_PI / 6);

  EXPECT_TRUE(halfPointState.toRotation().isApprox(expected.toRotation()));
  EXPECT_DOUBLE_EQ(expected.toAngle(), halfPointState.toAngle());
}

TEST(SO2, GeodesicInterpolationBeyondWrap)
{
  SO2::State from(2 * M_PI);
  SO2::State to(M_PI / 3.0);
  SO2::State toMinusFrom;
  SO2::State fromInv;

  SO2 so2;
  so2.getInverse(&from, &fromInv);
  so2.compose(&fromInv, &to, &toMinusFrom);

  Eigen::VectorXd tangentVector;
  so2.logMap(&toMinusFrom, tangentVector);

  SO2::State relativeState;
  so2.expMap(0.5 * tangentVector, &relativeState);

  SO2::State halfPointState;
  so2.compose(&from, &relativeState, &halfPointState);

  SO2::State expected(M_PI / 6.0);

  EXPECT_TRUE(halfPointState.toRotation().isApprox(expected.toRotation()));
  EXPECT_DOUBLE_EQ(expected.toAngle(), halfPointState.toAngle());
}

TEST(SO2, GeodesicInterpolationCornerCase)
{
  SO2::State from(M_PI);
  SO2::State to(-170 * M_PI / 180.0);
  SO2::State toMinusFrom;
  SO2::State fromInv;

  SO2 so2;
  so2.getInverse(&from, &fromInv);
  so2.compose(&fromInv, &to, &toMinusFrom);

  Eigen::VectorXd tangentVector;
  so2.logMap(&toMinusFrom, tangentVector);

  SO2::State relativeState;
  so2.expMap(0.5 * tangentVector, &relativeState);

  SO2::State halfPointState;
  so2.compose(&from, &relativeState, &halfPointState);

  SO2::State expected(-175.0 * M_PI / 180.0);

  EXPECT_TRUE(halfPointState.toRotation().isApprox(expected.toRotation()));
  EXPECT_DOUBLE_EQ(expected.toAngle(), halfPointState.toAngle());
}
