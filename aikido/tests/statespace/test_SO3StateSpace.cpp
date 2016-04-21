#include <gtest/gtest.h>
#include <aikido/statespace/SO3StateSpace.hpp>

using aikido::statespace::SO3StateSpace;

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
