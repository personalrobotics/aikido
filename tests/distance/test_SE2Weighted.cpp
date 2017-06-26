#include <aikido/distance/SE2Weighted.hpp>
#include <aikido/statespace/SE2.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(SE2WeightedDistance, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(SE2Weighted(nullptr), std::invalid_argument);

  Eigen::Vector2d empty;
  EXPECT_THROW(SE2Weighted(nullptr, empty), std::invalid_argument);
}