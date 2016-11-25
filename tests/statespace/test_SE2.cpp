#include <gtest/gtest.h>
#include <aikido/statespace/SE2.hpp>

using aikido::statespace::SE2;

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

  auto out = space.createState();
  space.compose(state2, state3, out);

  EXPECT_TRUE(expected_pose.isApprox(out.getIsometry()));
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
  se2.expMap(Eigen::Vector3d(M_PI_2, 0, 0), &out);

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
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI_2, 3, 4)));

  se2.expMap(Eigen::Vector3d(M_PI / 6, 4, 6), state);
  se2.logMap(state, out);
  EXPECT_TRUE(out.isApprox(Eigen::Vector3d(M_PI / 6, 4, 6)));
}

TEST(SE2, PrintState)
{
  SE2 se2;
  auto state = se2.createState();
  Eigen::Isometry2d pose1 = Eigen::Isometry2d::Identity();
  se2.print(state, std::cout);
}
