#include <aikido/distance/GeodesicDistanceMetric.hpp>
#include <aikido/statespace/SO3StateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(GeodesicDistance, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(GeodesicDistanceMetric(nullptr), std::invalid_argument);
}

TEST(GeodesicDistance, StateSpaceEquality)
{
  auto so3 = std::make_shared<SO3StateSpace>();
  GeodesicDistanceMetric dmetric(so3);
  EXPECT_EQ(so3, dmetric.getStateSpace());
}

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
