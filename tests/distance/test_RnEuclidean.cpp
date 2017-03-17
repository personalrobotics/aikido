#include <aikido/distance/RnEuclidean.hpp>
#include <aikido/statespace/Rn.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(RnEuclidean, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(RnEuclidean(nullptr), std::invalid_argument);
}

TEST(RnEuclidean, StateSpaceEquality)
{
  auto rvss = std::make_shared<Rn>(4);
  RnEuclidean dmetric(rvss);
  EXPECT_EQ(rvss, dmetric.getStateSpace());
}


TEST(RnEuclidean, Distance)
{
  auto rvss = std::make_shared<Rn>(4);
  auto state1 = rvss->createState();
  auto state2 = rvss->createState();

  state1.setValue(Eigen::Vector4d(0, 1, 2, 3));
  state2.setValue(Eigen::Vector4d(-1, -2, -3, -4));

  RnEuclidean dmetric(rvss);
  EXPECT_DOUBLE_EQ(std::sqrt(84), dmetric.distance(state1, state2));
}
