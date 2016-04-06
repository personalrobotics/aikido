// #include <cstdlib>
#include <gtest/gtest.h>
#include <aikido/util/VanDerCorput.hpp>

using aikido::util::VanDerCorput;

TEST(VanDerCorput, DefaultConstructorIncludesEndpoints)
{
  VanDerCorput vdc;
  EXPECT_DOUBLE_EQ(0.0, vdc());
}
