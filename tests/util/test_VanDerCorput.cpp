#include <gtest/gtest.h>
#include <dart/common/StlHelpers.hpp>
#include <aikido/util/VanDerCorput.hpp>

using aikido::util::VanDerCorput;

TEST(VanDerCorput, IncludesStartpoint)
{
  VanDerCorput vdc{1, true, false};
  EXPECT_DOUBLE_EQ(0.0, vdc[0].first);
  EXPECT_DOUBLE_EQ(0.5, vdc[1].first);
}

TEST(VanDerCorput, IncludesEndpoint)
{
  VanDerCorput vdc{1, false, true};
  EXPECT_DOUBLE_EQ(1.0, vdc[0].first);
  EXPECT_DOUBLE_EQ(0.5, vdc[1].first);
}

TEST(VanDerCorput, ExcludesEndpoints)
{
  VanDerCorput vdc{1, false, false};
  EXPECT_DOUBLE_EQ(0.5, vdc[0].first);
}

TEST(VanDerCorput, DefaultConstructorExcludesEndpoints)
{
  VanDerCorput vdc;
  EXPECT_DOUBLE_EQ(0.5, vdc[0].first);
}

TEST(VanDerCorput, FirstSevenValuesWithoutEndpoints)
{
  VanDerCorput vdc;
  EXPECT_DOUBLE_EQ(1./2, vdc[0].first);
  EXPECT_DOUBLE_EQ(1./4, vdc[1].first);
  EXPECT_DOUBLE_EQ(3./4, vdc[2].first);
  EXPECT_DOUBLE_EQ(1./8, vdc[3].first);
  EXPECT_DOUBLE_EQ(5./8, vdc[4].first);
  EXPECT_DOUBLE_EQ(3./8, vdc[5].first);
  EXPECT_DOUBLE_EQ(7./8, vdc[6].first);
}

TEST(VanDerCorput, FirstSevenIteratorValuesWithoutEndpoints)
{
  VanDerCorput vdc;
  auto itr = vdc.begin();
  EXPECT_DOUBLE_EQ(1./2, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(1./4, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(3./4, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(1./8, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(5./8, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(3./8, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(7./8, *itr);
}

TEST(VanDerCorput, FirstNineValuesWithEndpoints)
{
  VanDerCorput vdc{1, true, true};
  EXPECT_DOUBLE_EQ(0.0 , vdc[0].first);
  EXPECT_DOUBLE_EQ(1.0 , vdc[1].first);
  EXPECT_DOUBLE_EQ(1./2, vdc[2].first);
  EXPECT_DOUBLE_EQ(1./4, vdc[3].first);
  EXPECT_DOUBLE_EQ(3./4, vdc[4].first);
  EXPECT_DOUBLE_EQ(1./8, vdc[5].first);
  EXPECT_DOUBLE_EQ(5./8, vdc[6].first);
  EXPECT_DOUBLE_EQ(3./8, vdc[7].first);
  EXPECT_DOUBLE_EQ(7./8, vdc[8].first);
}

TEST(VanDerCorput, FirstNineIteratorValuesWithEndpoints)
{
  VanDerCorput vdc{1, true, true};
  auto itr = vdc.begin();
  EXPECT_DOUBLE_EQ(0.0 , *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(1.0 , *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(1./2, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(1./4, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(3./4, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(1./8, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(5./8, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(3./8, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(7./8, *itr);
}

TEST(VanDerCorput, ScaleProperlyOverSpan)
{
  VanDerCorput vdc{2};
  EXPECT_DOUBLE_EQ(2.0/2, vdc[0].first);
  EXPECT_DOUBLE_EQ(2.0/4, vdc[1].first);
  EXPECT_DOUBLE_EQ(6.0/4, vdc[2].first);
  EXPECT_DOUBLE_EQ(2.0/8, vdc[3].first);
  EXPECT_DOUBLE_EQ(10./8, vdc[4].first);
  EXPECT_DOUBLE_EQ(6.0/8, vdc[5].first);
  EXPECT_DOUBLE_EQ(14./8, vdc[6].first);
}

TEST(VanDerCorput, ScaleProperlyOverSpanWithEndpoints)
{
  VanDerCorput vdc{2, true, true};
  EXPECT_DOUBLE_EQ(0.0  , vdc[0].first);
  EXPECT_DOUBLE_EQ(2.0  , vdc[1].first);
  EXPECT_DOUBLE_EQ(2.0/2, vdc[2].first);
  EXPECT_DOUBLE_EQ(2.0/4, vdc[3].first);
  EXPECT_DOUBLE_EQ(6.0/4, vdc[4].first);
  EXPECT_DOUBLE_EQ(2.0/8, vdc[5].first);
  EXPECT_DOUBLE_EQ(10./8, vdc[6].first);
  EXPECT_DOUBLE_EQ(6.0/8, vdc[7].first);
  EXPECT_DOUBLE_EQ(14./8, vdc[8].first);
}

TEST(VanDerCorput, ResolutionAtEndpoints)
{
  VanDerCorput vdc{1, true, true};
  EXPECT_DOUBLE_EQ(1.0, vdc[0].second);
  EXPECT_DOUBLE_EQ(1.0, vdc[1].second);
}

TEST(VanDerCorput, ResolutionWhenAllDistancesTheSame)
{
  VanDerCorput vdc{1};
  EXPECT_DOUBLE_EQ(1./2, vdc[0].second);
  EXPECT_DOUBLE_EQ(1./4, vdc[2].second);
}

TEST(VanDerCorput, ResolutionAtAllPoints)
{
  VanDerCorput vdc{1};
  EXPECT_DOUBLE_EQ(1./2, vdc[0].second);
  EXPECT_DOUBLE_EQ(1./2, vdc[1].second);
  EXPECT_DOUBLE_EQ(1./4, vdc[2].second);
  EXPECT_DOUBLE_EQ(1./4, vdc[3].second);
  EXPECT_DOUBLE_EQ(1./4, vdc[4].second);
  EXPECT_DOUBLE_EQ(1./4, vdc[5].second);
  EXPECT_DOUBLE_EQ(1./8, vdc[6].second);
  EXPECT_DOUBLE_EQ(1./8, vdc[7].second);
  EXPECT_DOUBLE_EQ(1./8, vdc[8].second);
  EXPECT_DOUBLE_EQ(1./8, vdc[9].second);
  EXPECT_DOUBLE_EQ(1./8, vdc[10].second);
  EXPECT_DOUBLE_EQ(1./8, vdc[11].second);
  EXPECT_DOUBLE_EQ(1./8, vdc[12].second);
  EXPECT_DOUBLE_EQ(1./8, vdc[13].second);
  EXPECT_DOUBLE_EQ(1./16, vdc[14].second);
}

TEST(VanDerCorput, Iterable)
{
  VanDerCorput vdc{1, true, true, 0.5};
  int iterations = 0;
  EXPECT_NO_THROW({
    for (VanDerCorput::const_iterator itr = vdc.begin(); itr != vdc.end();
         ++itr) {
      ++iterations;
    }
  });
  EXPECT_EQ(3, iterations);
}

TEST(VanDerCorput, ForEachIterable)
{
  VanDerCorput vdc{1, true, true, 0.5};
  int iterations = 0;
  EXPECT_NO_THROW({
    for (double d : vdc) {
      DART_UNUSED(d);
      ++iterations;
    }
  });
  EXPECT_EQ(3, iterations);
}

TEST(VanDerCorput, IterationEndsWhenMinimumResolutionReached)
{
  VanDerCorput vdc_0_5{1, false, false, 0.5};
  int iterations = 0;
  for (auto itr = vdc_0_5.begin(); itr != vdc_0_5.end(); ++itr) {
    ++iterations;
  }
  EXPECT_EQ(1, iterations);
  EXPECT_EQ(1, vdc_0_5.getLength());

  VanDerCorput vdc_0_25{1, false, false, 0.25};
  iterations = 0;
  for (auto itr = vdc_0_25.begin(); itr != vdc_0_25.end(); ++itr) {
    ++iterations;
  }
  EXPECT_EQ(3, iterations);
  EXPECT_EQ(3, vdc_0_25.getLength());

  VanDerCorput vdc_0_125{1, false, false, 0.125};
  iterations = 0;
  for (auto itr = vdc_0_125.begin(); itr != vdc_0_125.end(); ++itr) {
    ++iterations;
  }
  EXPECT_EQ(7, iterations);
  EXPECT_EQ(7, vdc_0_125.getLength());
}
