#include <aikido/distance/GeodesicDistanceMetric.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(GeodesicDistance, Distance)
{
  auto so3 = std::make_shared<SO3StateSpace>();
  GeodesicDistanceMetric dmetric(so3);
  auto state1 = so3->createState();
  auto state2 = so3->createState();

  auto quat =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
  state1.setQuaternion(quat);
  state2.setQuaternion(quat);
  EXPECT_DOUBLE_EQ(0.0, dmetric.distance(state1, state2));

  auto quat2 = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.5 + M_PI, Eigen::Vector3d::UnitY()));
  state2.setQuaternion(quat2);
  EXPECT_NEAR(0.5, dmetric.distance(state1, state2), 1e-6);
}

TEST(GeodesicDistance, Interpolate)
{
  auto so3 = std::make_shared<SO3StateSpace>();
  GeodesicDistanceMetric dmetric(so3);
  auto state1 = so3->createState();
  auto state2 = so3->createState();
  auto istate = so3->createState();

  auto quat1 =
      Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()));
  auto quat2 = Eigen::Quaterniond(
      Eigen::AngleAxisd(1.0 + M_PI, Eigen::Vector3d::UnitY()));
  auto quat3 = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.5 + M_PI, Eigen::Vector3d::UnitY()));

  state1.setQuaternion(quat1);
  state2.setQuaternion(quat2);

  dmetric.interpolate(state1, state2, 0, istate);
  EXPECT_TRUE(istate.getQuaternion().isApprox(quat1));

  dmetric.interpolate(state1, state2, 1, istate);
  EXPECT_TRUE(istate.getQuaternion().isApprox(quat2));

  dmetric.interpolate(state1, state2, 0.5, istate);
  EXPECT_TRUE(istate.getQuaternion().isApprox(quat3));
}
