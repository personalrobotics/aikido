#include <gtest/gtest.h>
#include <aikido/statespace/SE3.hpp>

using aikido::statespace::SE3;
using Vector6d = Eigen::Matrix<double, 6, 1>;

TEST(SE3, Compose)
{
  SE3 space;

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

  SE3::ScopedState out(&space);
  space.compose(s2, s3, out);

  EXPECT_TRUE(expected.isApprox(out.getIsometry()));
}

TEST(SE3, Identity)
{
  SE3 space;

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

TEST(SE3, Inverse)
{
  SE3 space;

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

TEST(SE3, ExpMap)
{
  SE3::State out;

  Eigen::Isometry3d expected_pose = Eigen::Isometry3d::Identity();
  expected_pose.translation() = Eigen::Vector3d(1, 2, 3);

  SE3 se3;
  Vector6d twist(Vector6d::Zero());
  twist.tail<3>() = Eigen::Vector3d(1, 2, 3);

  se3.expMap(twist, &out);

  EXPECT_TRUE(out.getIsometry().isApprox(expected_pose));
}

TEST(SE3, LogMap)
{
  SE3 se3;
  Vector6d twist;
  twist << M_PI / 5, 0, 0, 1, 2, 3;

  auto state = se3.createState();
  Eigen::VectorXd out;
  se3.expMap(twist, state);
  se3.logMap(state, out);
  EXPECT_TRUE(out.isApprox(twist));
}

TEST(SE3, PrintState)
{
  SE3 se3;
  auto state = se3.createState();
  state.setIsometry(Eigen::Isometry3d::Identity());
  se3.print(state, std::cout);
}
