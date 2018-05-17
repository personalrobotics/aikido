#include <dart/math/Helpers.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/SO2.hpp>
#include "eigen_tests.hpp"

using aikido::statespace::SO2;
using aikido::tests::make_vector;
using Eigen::Rotation2Dd;

static constexpr double TOLERANCE{1e-6};

TEST(SO2, Clone)
{
  SO2 so2;

  for (auto i = 0u; i < 5u; ++i)
  {
    const auto angle = dart::math::random(-M_PI, M_PI);

    auto s1 = so2.createState();
    s1.setAngle(angle);

    auto s2 = s1.clone();

    EXPECT_DOUBLE_EQ(s1.getAngle(), s2.getAngle());
  }
}

TEST(SO2, Compose)
{
  SO2::State s1(M_PI / 4);
  SO2::State s2(M_PI / 2);
  SO2::State out1;
  SO2::State out2;
  SO2::State expected(3.0 / 4.0 * M_PI);

  SO2 so2;

  so2.compose(&s1, &s2, &out1);
  EXPECT_TRUE(out1.getRotation().isApprox(expected.getRotation()));

  so2.copyState(&s1, &out2);
  so2.compose(&out2, &s2);
  EXPECT_TRUE(out2.getRotation().isApprox(expected.getRotation()));
}

TEST(SO2, Identity)
{
  SO2 so2;
  auto s1 = so2.createState();
  s1.setAngle(M_PI / 4);

  auto ident = so2.createState();
  so2.getIdentity(ident);

  auto out = so2.createState();
  so2.compose(s1, ident, out);
  EXPECT_DOUBLE_EQ(s1.getAngle(), out.getAngle());
}

TEST(SO2, Inverse)
{
  SO2 so2;
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

TEST(SO2, ExpMap)
{
  SO2::State out;
  SO2::State expected;

  SO2 so2;

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

TEST(SO2, LogMap)
{
  SO2::State state;
  SO2 so2;
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

TEST(SO2, CopyState)
{
  SO2 so2;
  auto dest = so2.createState();
  auto source = so2.createState();
  source.setAngle(3.14159);
  so2.copyState(source, dest);
  EXPECT_DOUBLE_EQ(source.getAngle(), dest.getAngle());
}

TEST(SO2, PrintState)
{
  SO2 so2;
  auto source = so2.createState();
  source.setAngle(M_PI);
  so2.print(source, std::cout);
}
