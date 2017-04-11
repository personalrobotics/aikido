#include <gtest/gtest.h>
#include <dart/common/StlHelpers.hpp>
#include <aikido/util/StepSequence.hpp>

using aikido::util::StepSequence;

TEST(StepSequence, IncludesEndpoints)
{
  StepSequence seq(0.55, true);
  EXPECT_DOUBLE_EQ(0.0, seq[-1]);
  EXPECT_DOUBLE_EQ(1.0, seq[2]);
}

TEST(StepSequence, DefaultConstructorIncludesEndpoints)
{
  StepSequence seq(0.55);
  EXPECT_DOUBLE_EQ(0.0, seq[-1]);
  EXPECT_DOUBLE_EQ(1.0, seq[2]);
}

TEST(StepSequence, ExcludesEndpoints)
{
  StepSequence seq(0.55, false);
  EXPECT_THROW(seq[-1], std::out_of_range);
  EXPECT_THROW(seq[2], std::out_of_range);
}

TEST(StepSequence, Iterator)
{
  StepSequence seq(0.2, true);
  auto itr = seq.begin();
  EXPECT_DOUBLE_EQ(0, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(0.2, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(0.4, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(0.6, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(0.8, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(1.0, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(1.0, *itr);
}

TEST(StepSequence, IteratorAlternateRange)
{
  StepSequence seq(0.2, true, 24, 24.5);
  auto itr = seq.begin();
  EXPECT_DOUBLE_EQ(24, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(24.2, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(24.4, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(24.5, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(24.5, *itr);
  ++itr;
  EXPECT_DOUBLE_EQ(24.5, *itr);
}

TEST(StepSequence, IteratorStop)
{
  size_t count = 0;
  StepSequence seq(0.3, true);
  for (double v : seq) {
    DART_UNUSED(v);
    count++;
  }

  EXPECT_EQ(5, count);
}

TEST(StepSequence, IteratorStopAlternateRange)
{
  size_t count = 0;
  StepSequence seq(0.3, true, 3, 3.7);
  for (double v : seq) {
    DART_UNUSED(v);
    count++;
  }

  EXPECT_EQ(4, count);
}

TEST(StepSequence, MaxSteps)
{
  StepSequence seq1(0.2, true);
  EXPECT_EQ(6, seq1.getMaxSteps());

  StepSequence seq2(0.2, false);
  EXPECT_EQ(6, seq2.getMaxSteps());

  StepSequence seq3(0.3, true);
  EXPECT_EQ(5, seq3.getMaxSteps());

  StepSequence seq4(0.3, false);
  EXPECT_EQ(4, seq4.getMaxSteps());
}

TEST(StepSequence, MaxStepsAlternateRanges)
{
  StepSequence seq1(0.2, true, 3, 7);
  EXPECT_EQ(21, seq1.getMaxSteps());

  StepSequence seq2(0.2, false, 3, 7);
  EXPECT_EQ(21, seq2.getMaxSteps());

  StepSequence seq3(0.3, true, 2, 4);
  EXPECT_EQ(8, seq3.getMaxSteps());

  StepSequence seq4(0.3, false, 2, 5);
  EXPECT_EQ(11, seq4.getMaxSteps());
}
