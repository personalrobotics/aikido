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
  EXPECT_DOUBLE_EQ(1./2, vdc());
  EXPECT_DOUBLE_EQ(1./4, vdc());
  EXPECT_DOUBLE_EQ(3./4, vdc());
  EXPECT_DOUBLE_EQ(1./8, vdc());
  EXPECT_DOUBLE_EQ(5./8, vdc());
  EXPECT_DOUBLE_EQ(3./8, vdc());
  EXPECT_DOUBLE_EQ(7./8, vdc());
}
  
TEST(VanDerCorput, FirstFiveValuesWithEndpoints)
{
  VanDerCorput vdc{1, true};
  EXPECT_DOUBLE_EQ(0.0, vdc());
  EXPECT_DOUBLE_EQ(1.0, vdc());
  EXPECT_DOUBLE_EQ(1./2, vdc());
  EXPECT_DOUBLE_EQ(1./4, vdc());
  EXPECT_DOUBLE_EQ(3./4, vdc());
  EXPECT_DOUBLE_EQ(1./8, vdc());
  EXPECT_DOUBLE_EQ(5./8, vdc());
  EXPECT_DOUBLE_EQ(3./8, vdc());
  EXPECT_DOUBLE_EQ(7./8, vdc());
}

TEST(VanDerCorput, ScaleProperlyOverSpanWithEndpoints)
{
  VanDerCorput vdc{2, true};
  EXPECT_DOUBLE_EQ(0.0, vdc());
  EXPECT_DOUBLE_EQ(2.0, vdc());
  EXPECT_DOUBLE_EQ(2.0/2, vdc());
  EXPECT_DOUBLE_EQ(2.0/4, vdc());
  EXPECT_DOUBLE_EQ(6.0/4, vdc());
  EXPECT_DOUBLE_EQ(2.0/8, vdc());
  EXPECT_DOUBLE_EQ(10./8, vdc());
  EXPECT_DOUBLE_EQ(6.0/8, vdc());
  EXPECT_DOUBLE_EQ(14./8, vdc());
} 

TEST(VanDerCorput, ResolutionAtEndpoints)
{
  VanDerCorput vdc{1, true};
  vdc();
  EXPECT_DOUBLE_EQ(1.0, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1.0, vdc.current_resolution());
}

TEST(VanDerCorput, ResolutionWhenAllDistancesTheSame)
{
  VanDerCorput vdc{1, false};
  vdc();
  EXPECT_DOUBLE_EQ(1./2, vdc.current_resolution());
  vdc();
  vdc();
  EXPECT_DOUBLE_EQ(1./4, vdc.current_resolution());
}

TEST(VanDerCorput, ResolutionAtAllPoints)
{
  VanDerCorput vdc{1, false};
  vdc();
  EXPECT_DOUBLE_EQ(1./2, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./2, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./4, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./4, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./4, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./4, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./8, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./8, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./8, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./8, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./8, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./8, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./8, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./8, vdc.current_resolution());
  vdc();
  EXPECT_DOUBLE_EQ(1./16, vdc.current_resolution());
}
