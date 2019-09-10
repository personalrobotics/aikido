#include <gtest/gtest.h>

#include <aikido/distance/RnEuclidean.hpp>
#include <aikido/statespace/Rn.hpp>

using namespace aikido::distance;
using namespace aikido::statespace;

using R4 = R<4>;
using R4Euclidean = REuclidean<4>;

//==============================================================================
TEST(REuclidean, ThrowsOnNullStateSpaceR1)
{
  EXPECT_THROW(R1Euclidean(nullptr), std::invalid_argument);
}

//==============================================================================
TEST(REuclidean, ThrowsOnNullStateSpaceRx)
{
  EXPECT_THROW(RnEuclidean(nullptr), std::invalid_argument);
}

//==============================================================================
TEST(REuclidean, StateSpaceEqualityR4)
{
  auto rvss = std::make_shared<R4>();
  R4Euclidean dmetric(rvss);
  EXPECT_EQ(rvss, dmetric.getStateSpace());
}

//==============================================================================
TEST(REuclidean, StateSpaceEqualityRx)
{
  auto rvss = std::make_shared<Rn>(4);
  RnEuclidean dmetric(rvss);
  EXPECT_EQ(rvss, dmetric.getStateSpace());
}

//==============================================================================
TEST(REuclidean, DistanceR4)
{
  auto rvss = std::make_shared<R4>();
  auto state1 = rvss->createState();
  auto state2 = rvss->createState();

  state1.setValue(Eigen::Vector4d(0, 1, 2, 3));
  state2.setValue(Eigen::Vector4d(-1, -2, -3, -4));

  R4Euclidean dmetric(rvss);
  EXPECT_DOUBLE_EQ(std::sqrt(84), dmetric.distance(state1, state2));
}

//==============================================================================
TEST(REuclidean, DistanceRx)
{
  auto rvss = std::make_shared<Rn>(4);
  auto state1 = rvss->createState();
  auto state2 = rvss->createState();

  state1.setValue(Eigen::Vector4d(0, 1, 2, 3));
  state2.setValue(Eigen::Vector4d(-1, -2, -3, -4));

  RnEuclidean dmetric(rvss);
  EXPECT_DOUBLE_EQ(std::sqrt(84), dmetric.distance(state1, state2));
}
