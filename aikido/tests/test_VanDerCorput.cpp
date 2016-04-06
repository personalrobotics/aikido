// #include <cstdlib>
#include <gtest/gtest.h>
#include <aikido/util/VanDerCorput.hpp>

using aikido::util::VanDerCorput;

TEST(VanDerCorput, IncludesEndpoints)
{
  VanDerCorput vdc{1, true};
  EXPECT_DOUBLE_EQ(0.0, vdc());
  EXPECT_DOUBLE_EQ(1.0, vdc());
}

TEST(VanDerCorput, ExcludesEndpoints)
{
  VanDerCorput vdc{1, false};
  EXPECT_DOUBLE_EQ(0.5, vdc());
}

TEST(VanDerCorput, DefaultConstructorExcludesEndpoints)
{
  VanDerCorput vdc;
  EXPECT_DOUBLE_EQ(0.5, vdc());
}

TEST(VanDerCorput, FirstThreeValuesWithoutEndpoints)
{
  VanDerCorput vdc{1, false};
  EXPECT_DOUBLE_EQ(0.5, vdc());
  EXPECT_DOUBLE_EQ(0.25, vdc());
  EXPECT_DOUBLE_EQ(0.75, vdc());
}
  
TEST(VanDerCorput, FirstFiveValuesWithEndpoints)
{
  VanDerCorput vdc{1, true};
  EXPECT_DOUBLE_EQ(0.0, vdc());
  EXPECT_DOUBLE_EQ(1.0, vdc());
  EXPECT_DOUBLE_EQ(0.5, vdc());
  EXPECT_DOUBLE_EQ(0.25, vdc());
  EXPECT_DOUBLE_EQ(0.75, vdc());
}

TEST(VanDerCorput, ScaleProperlyOverSpanWithEndpoints)
{
  VanDerCorput vdc{2, true};
  EXPECT_DOUBLE_EQ(0.0, vdc());
  EXPECT_DOUBLE_EQ(2.0, vdc());
  EXPECT_DOUBLE_EQ(1.0, vdc());
  EXPECT_DOUBLE_EQ(0.5, vdc());
  EXPECT_DOUBLE_EQ(1.5, vdc());
} 
