#include <aikido/distance/AngularDistanceMetric.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(AngularDistanceMetric, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(AngularDistanceMetric(nullptr), std::invalid_argument);
}

TEST(AngularDistanceMetric, StateSpaceEquality)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  AngularDistanceMetric dmetric(so2);

  EXPECT_EQ(so2, dmetric.getStateSpace());
}

TEST(AngularDistanceMetric, Distance)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  AngularDistanceMetric dmetric(so2);

  auto state1 = so2->createState();
  auto state2 = so2->createState();
  state1.setAngle(1.3);
  state2.setAngle(1.5);
  EXPECT_DOUBLE_EQ(0.2, dmetric.distance(state1, state2));

  state2.setAngle(6.0);
  EXPECT_DOUBLE_EQ(2. * M_PI - state2.getAngle() + state1.getAngle(),
                   dmetric.distance(state1, state2));
}
