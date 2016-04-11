#include <aikido/distance/AngularDistanceMetric.hpp>
#include <aikido/statespace/SO2StateSpace.hpp>

#include <gtest/gtest.h>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(SO2StateSpace, Distance)
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

TEST(SO2StateSpace, Interpolate)
{
  auto so2 = std::make_shared<SO2StateSpace>();
  AngularDistanceMetric dmetric(so2);

  auto state1 = so2->createState();
  auto state2 = so2->createState();
  auto out = so2->createState();

  state1.setAngle(0.);
  state2.setAngle(2. * M_PI);
  dmetric.interpolate(state1, state2, 0, out);
  EXPECT_DOUBLE_EQ(state1.getAngle(), out.getAngle());

  dmetric.interpolate(state1, state2, 1, out);
  EXPECT_DOUBLE_EQ(state2.getAngle(), out.getAngle());

  dmetric.interpolate(state1, state2, 0.25, out);
  EXPECT_DOUBLE_EQ(M_PI_2, out.getAngle());
}
