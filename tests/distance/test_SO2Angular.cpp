#include <gtest/gtest.h>

#include <aikido/distance/SO2Angular.hpp>
#include <aikido/statespace/SO2.hpp>

using namespace aikido::distance;
using namespace aikido::statespace;

TEST(SO2Angular, ThrowsOnNullStateSpace)
{
  EXPECT_THROW(SO2Angular(nullptr), std::invalid_argument);
}

TEST(SO2Angular, StateSpaceEquality)
{
  auto so2 = std::make_shared<SO2>();
  SO2Angular dmetric(so2);

  EXPECT_EQ(so2, dmetric.getStateSpace());
}

TEST(SO2Angular, Distance)
{
  auto so2 = std::make_shared<SO2>();
  SO2Angular dmetric(so2);

  auto state1 = so2->createState();
  auto state2 = so2->createState();
  state1.fromAngle(1.3);
  state2.fromAngle(1.5);
  EXPECT_DOUBLE_EQ(0.2, dmetric.distance(state1, state2));

  state2.fromAngle(6.0);
  EXPECT_DOUBLE_EQ(
      state1.toAngle() - state2.toAngle(), dmetric.distance(state1, state2));

  state1.fromAngle(3.0);
  state2.fromAngle(-3.0);
  EXPECT_DOUBLE_EQ(
      2 * M_PI - state1.toAngle() + state2.toAngle(),
      dmetric.distance(state1, state2));
}
