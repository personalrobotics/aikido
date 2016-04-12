#include <aikido/distance/EuclideanDistanceMetric.hpp>
#include <aikido/statespace/RealVectorStateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(EuclideanDistanceMetric, Distance)
{
  auto rvss = std::make_shared<RealVectorStateSpace>(4);
  auto state1 = rvss->createState();
  auto state2 = rvss->createState();

  state1.setValue(Eigen::Vector4d(0, 1, 2, 3));
  state2.setValue(Eigen::Vector4d(-1, -2, -3, -4));

  EuclideanDistanceMetric dmetric(rvss);
  EXPECT_DOUBLE_EQ(std::sqrt(84), dmetric.distance(state1, state2));
}

TEST(EuclideanDistanceMetric, Interpolate)
{
  auto rvss = std::make_shared<RealVectorStateSpace>(4);
  EuclideanDistanceMetric dmetric(rvss);

  auto state1 = rvss->createState();
  auto state2 = rvss->createState();
  auto out = rvss->createState();

  state1.setValue(Eigen::Vector4d(0, 1, 2, 3));
  state2.setValue(Eigen::Vector4d(-1, -2, -3, -4));

  dmetric.interpolate(state1, state2, 0, out);
  EXPECT_TRUE(out.getValue().isApprox(state1.getValue()));

  dmetric.interpolate(state1, state2, 1, out);
  EXPECT_TRUE(out.getValue().isApprox(state2.getValue()));

  dmetric.interpolate(state1, state2, 0.5, out);
  EXPECT_TRUE(out.getValue().isApprox(Eigen::Vector4d(-0.5, -0.5, -0.5, -0.5)));

  EXPECT_ANY_THROW(dmetric.interpolate(state1, state2, -0.5, out));
  EXPECT_ANY_THROW(dmetric.interpolate(state1, state2, 1.1, out));
}
