#include <dart/math/Helpers.hpp>
#include <gtest/gtest.h>
#include <aikido/statespace/SE2.hpp>

using aikido::statespace::SE2;

TEST(SE2, Clone)
{
  SE2 se2;

  for (auto i = 0u; i < 5u; ++i)
  {
#if DART_VERSION_AT_LEAST(6, 7, 0)
    const auto angle = dart::math::Random::uniform(-M_PI, M_PI);
#else
    const auto angle = dart::math::random(-M_PI, M_PI);
#endif
    Eigen::Isometry2d pose = Eigen::Isometry2d::Identity();
    pose.rotate(Eigen::Rotation2Dd(angle));
    pose.translation() = Eigen::Vector2d::Random();

    auto s1 = se2.createState();
    s1.setIsometry(pose);

    auto s2 = s1.clone();

    EXPECT_TRUE(s1.getIsometry().isApprox(s2.getIsometry()));
  }
}

TEST(SE2, Compose)
{
  SE2 space;

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

  auto out1 = space.createState();
  space.compose(state2, state3, out1); // out1 = state2 * state3
  EXPECT_TRUE(expected_pose.isApprox(out1.getIsometry()));

  auto out2 = space.createState();
  space.copyState(state2, out2); // out2 = state2
  space.compose(out2, state3);   // out2 = out2 * state3
  EXPECT_TRUE(expected_pose.isApprox(out2.getIsometry()));
}

TEST(SE2, Identity)
{
  SE2 space;
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

TEST(SE2, Inverse)
{
  SE2 space;
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

TEST(SE2, ExpMap)
{
  SE2::State out;

  Eigen::Isometry2d expected_pose = Eigen::Isometry2d::Identity();
  expected_pose.rotate(Eigen::Rotation2Dd(M_PI_2));

  SE2 se2;
  se2.expMap(Eigen::Vector3d(0, 0, M_PI_2), &out);

  EXPECT_TRUE(out.getIsometry().isApprox(expected_pose));
}

TEST(SE2, LogMap)
{
  SE2 se2;
  auto state = se2.createState();
  Eigen::Isometry2d pose1 = Eigen::Isometry2d::Identity();
  pose1.rotate(Eigen::Rotation2Dd(M_PI_2));
  pose1.translation() = Eigen::Vector2d(3, 4);
  se2.setIsometry(state, pose1);

  Eigen::VectorXd out;
  se2.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(3, 4, M_PI_2)));

  se2.expMap(Eigen::Vector3d(4, 6, M_PI / 6), state);
  se2.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(4, 6, M_PI / 6)));
}

TEST(SE2, PrintState)
{
  SE2 se2;
  auto state = se2.createState();
  se2.print(state, std::cout);
}
