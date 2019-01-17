#include <dart/math/Helpers.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/SO3.hpp>

using aikido::statespace::SO3;

TEST(SO3, Clone)
{
  SO3 so3;

  for (auto i = 0u; i < 5u; ++i)
  {
#if DART_VERSION_AT_LEAST(6, 7, 0)
    const auto angle = dart::math::Random::uniform(-M_PI, M_PI);
#else
    const auto angle = dart::math::random(-M_PI, M_PI);
#endif
    const auto axis = Eigen::Vector3d::Random().normalized();
    const auto angleAxis = Eigen::AngleAxisd(angle, axis);
    const Eigen::Quaterniond quat(angleAxis);

    auto s1 = so3.createState();
    s1.setQuaternion(quat);

    auto s2 = s1.clone();

    EXPECT_TRUE(s1.getQuaternion().isApprox(s2.getQuaternion()));
  }
}

TEST(SO3, Compose)
{
  SO3::State identity;
  EXPECT_TRUE(
      identity.getQuaternion().isApprox(Eigen::Quaterniond::Identity()));

  SO3::State s2(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3::State s3(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3::State expected(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));

  SO3::State out1;
  SO3::State out2;
  SO3 so3;

  so3.compose(&s2, &s3, &out1);
  EXPECT_TRUE(expected.getQuaternion().isApprox(out1.getQuaternion()));

  so3.copyState(&s2, &out2);
  so3.compose(&out2, &s3);
  EXPECT_TRUE(expected.getQuaternion().isApprox(out2.getQuaternion()));
}

TEST(SO3, Identity)
{
  SO3 so3;
  SO3::State s1(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3::State ident;
  so3.getIdentity(&ident);

  SO3::State out;
  so3.compose(&s1, &ident, &out);

  EXPECT_TRUE(s1.getQuaternion().isApprox(out.getQuaternion()));
}

TEST(SO3, Inverse)
{
  SO3 so3;
  SO3::State s1(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
  SO3::State ident;
  so3.getIdentity(&ident);
  SO3::State inv;
  so3.getInverse(&s1, &inv);

  SO3::State out;
  so3.compose(&s1, &inv, &out);

  EXPECT_TRUE(ident.getQuaternion().isApprox(out.getQuaternion()));
}

TEST(SO3, ExpMap)
{
  SO3::State out;
  SO3::State expected(
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));

  SO3 so3;
  so3.expMap(Eigen::Vector3d(0, 0, M_PI_2), &out);

  EXPECT_TRUE(out.getQuaternion().isApprox(expected.getQuaternion()));
}

TEST(SO3, LogMap)
{
  SO3 so3;
  auto state = so3.createState();
  so3.setQuaternion(
      state,
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));

  Eigen::VectorXd out;
  so3.logMap(state, out);
  EXPECT_TRUE(Eigen::Vector3d(0, 0, M_PI_2).isApprox(out));

  so3.expMap(Eigen::Vector3d(M_PI_2, M_PI_2, M_PI / 5), state);
  so3.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI_2, M_PI_2, M_PI / 5)));
}

TEST(SO3, CopyState)
{
  SO3 so3;
  auto source = so3.createState();
  auto dest = so3.createState();

  auto quat = Eigen::Quaterniond(
      Eigen::AngleAxisd(1.8 * M_PI, Eigen::Vector3d::UnitX()));
  source.setQuaternion(quat);

  so3.copyState(source, dest);
  EXPECT_TRUE(source.getQuaternion().isApprox(dest.getQuaternion()));
}

TEST(SO3, PrintState)
{
  SO3 so3;
  auto source = so3.createState();
  auto quat = Eigen::Quaterniond(
      Eigen::AngleAxisd(1.8 * M_PI, Eigen::Vector3d::UnitX()));
  source.setQuaternion(quat);
  so3.print(source, std::cout);
}
