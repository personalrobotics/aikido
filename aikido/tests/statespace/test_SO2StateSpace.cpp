#include <gtest/gtest.h>
#include <aikido/statespace/SO2StateSpace.hpp>
#include "eigen_tests.hpp"

using aikido::statespace::SO2StateSpace;
using aikido::tests::make_vector;
using Eigen::Rotation2Dd;

static constexpr double TOLERANCE { 1e-6 };

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

  so2.expMap(make_vector(0.), &out);
  EXPECT_EIGEN_EQUAL(
    Rotation2Dd(0.).matrix(), expected.getRotation().matrix(), TOLERANCE);

  so2.expMap(make_vector(2 * M_PI), &out);
  EXPECT_EIGEN_EQUAL(
    Rotation2Dd(0.).matrix(), out.getRotation().matrix(), TOLERANCE);

  so2.expMap(make_vector(-3 * M_PI), &out);
  EXPECT_EIGEN_EQUAL(
    Rotation2Dd(M_PI).matrix(), out.getRotation().matrix(), TOLERANCE);
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

