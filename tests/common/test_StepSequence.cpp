#include <gtest/gtest.h>

#include <aikido/common/StepSequence.hpp>
#include <aikido/common/memory.hpp>

using aikido::common::StepSequence;

TEST(StepSequence, Constructor)
{
  EXPECT_THROW(StepSequence(0.2, true, true, 1.0, 0.0), std::runtime_error);
  EXPECT_THROW(StepSequence(-0.2, true, true, 0.0, 1.0), std::runtime_error);
  EXPECT_THROW(StepSequence(0.0), std::runtime_error);
}

TEST(StepSequence, GetLength)
{
  StepSequence seq11(0.2, true, true);
  // [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
  EXPECT_EQ(6, seq11.getLength());
  StepSequence seq12(0.2, true, false);
  // [0.0, 0.2, 0.4, 0.6, 0.8]
  EXPECT_EQ(5, seq12.getLength());
  StepSequence seq13(0.2, false, true);
  // [0.2, 0.4, 0.6, 0.8, 1.0]
  EXPECT_EQ(5, seq13.getLength());
  StepSequence seq14(0.2, false, false);
  // [0.2, 0.4, 0.6, 0.8]
  EXPECT_EQ(4, seq14.getLength());

  StepSequence seq21(0.3, true, true);
  // [0.0, 0.3, 0.6, 0.9, 1.0]
  EXPECT_EQ(5, seq21.getLength());
  StepSequence seq22(0.3, true, false);
  // [0.0, 0.3, 0.6, 0.9]
  EXPECT_EQ(4, seq22.getLength());
  StepSequence seq23(0.3, false, true);
  // [0.3, 0.6, 0.9, 1.0]
  EXPECT_EQ(4, seq23.getLength());
  StepSequence seq24(0.3, false, false);
  // [0.3, 0.6, 0.9]
  EXPECT_EQ(3, seq24.getLength());

  StepSequence seq31(1.2, true, true);
  // [0.0, 1.0]
  EXPECT_EQ(2, seq31.getLength());
  StepSequence seq32(1.2, true, false);
  // [0.0]
  EXPECT_EQ(1, seq32.getLength());
  StepSequence seq33(1.2, false, true);
  // [1.0]
  EXPECT_EQ(1, seq33.getLength());
  StepSequence seq34(1.2, false, false);
  // []
  EXPECT_EQ(0, seq34.getLength());

  StepSequence seq41(0.2, true, true, 0.0, 0.0);
  // [0.0]
  EXPECT_EQ(1, seq41.getLength());
  StepSequence seq42(0.2, true, false, 0.0, 0.0);
  // []
  EXPECT_EQ(0, seq42.getLength());
  StepSequence seq43(0.2, false, true, 0.0, 0.0);
  // []
  EXPECT_EQ(0, seq43.getLength());
  StepSequence seq44(0.2, false, false, 0.0, 0.0);
  // []
  EXPECT_EQ(0, seq44.getLength());

  StepSequence seq51(0.2, true, true, 0.2, 0.4);
  // [0.2, 0.4]
  EXPECT_EQ(2, seq51.getLength());
  StepSequence seq52(0.2, true, false, 0.2, 0.4);
  // [0.2]
  EXPECT_EQ(1, seq52.getLength());
  StepSequence seq53(0.2, false, true, 0.2, 0.4);
  // [0.4]
  EXPECT_EQ(1, seq53.getLength());
  StepSequence seq54(0.2, false, false, 0.2, 0.4);
  // []
  EXPECT_EQ(0, seq54.getLength());

  StepSequence seq61(0.2, true, true, -1.0, 0.0);
  // [-1.0 -0.8 -0.6 -0.4 -0.2 0.0]
  EXPECT_EQ(6, seq61.getLength());
  StepSequence seq62(0.2, true, false, -1.0, 0.0);
  // [-1.0 -0.8 -0.6 -0.4 -0.2]
  EXPECT_EQ(5, seq62.getLength());
  StepSequence seq63(0.2, false, true, -1.0, 0.0);
  // [-0.8 -0.6 -0.4 -0.2 0.0]
  EXPECT_EQ(5, seq63.getLength());
  StepSequence seq64(0.2, false, false, -1.0, 0.0);
  // [-0.8 -0.6 -0.4 -0.2]
  EXPECT_EQ(4, seq64.getLength());

  StepSequence seq71(-0.2, true, true, 1.0, 0.0);
  // [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
  EXPECT_EQ(6, seq71.getLength());
  StepSequence seq72(-0.2, true, false, 1.0, 0.0);
  // [0.0, 0.2, 0.4, 0.6, 0.8]
  EXPECT_EQ(5, seq72.getLength());
  StepSequence seq73(-0.2, false, true, 1.0, 0.0);
  // [0.2, 0.4, 0.6, 0.8, 1.0]
  EXPECT_EQ(5, seq73.getLength());
  StepSequence seq74(-0.2, false, false, 1.0, 0.0);
  // [0.2, 0.4, 0.6, 0.8]
  EXPECT_EQ(4, seq74.getLength());
}

TEST(StepSequence, MaxStepsAlternateRanges)
{
  StepSequence seq11(0.2, true, true, 3, 7);
  EXPECT_EQ(21, seq11.getLength());

  StepSequence seq12(0.2, true, false, 3, 7);
  EXPECT_EQ(20, seq12.getLength());

  StepSequence seq13(0.2, false, true, 3, 7);
  EXPECT_EQ(20, seq13.getLength());

  StepSequence seq14(0.2, false, false, 3, 7);
  EXPECT_EQ(19, seq14.getLength());

  StepSequence seq21(0.3, true, true, 2, 4);
  EXPECT_EQ(8, seq21.getLength());

  StepSequence seq22(0.3, true, false, 2, 4);
  EXPECT_EQ(7, seq22.getLength());

  StepSequence seq23(0.3, false, true, 2, 4);
  EXPECT_EQ(7, seq23.getLength());

  StepSequence seq24(0.3, false, false, 2, 4);
  EXPECT_EQ(6, seq24.getLength());
}

TEST(StepSequence, IncludesStartPoint)
{
  StepSequence seq(0.55, true, false);
  // suport index [0, 1]
  // [0.0, 0.55]
  EXPECT_THROW(seq[-1], std::out_of_range);
  EXPECT_THROW(seq[2], std::out_of_range);
  EXPECT_DOUBLE_EQ(0.0, seq[0]);
  EXPECT_DOUBLE_EQ(0.55, seq[1]);

  StepSequence seq2(0.5, true, false);
  // suport index [0, 1]
  // [0.0, 0.5]
  EXPECT_THROW(seq2[-1], std::out_of_range);
  EXPECT_THROW(seq2[2], std::out_of_range);
  EXPECT_DOUBLE_EQ(0.0, seq2[0]);
  EXPECT_DOUBLE_EQ(0.5, seq2[1]);
}

TEST(StepSequence, IncludesEndpoint)
{
  StepSequence seq(0.55, false, true);
  // suport index [0, 1]
  // [0.55, 1.0]
  EXPECT_THROW(seq[-1], std::out_of_range);
  EXPECT_THROW(seq[2], std::out_of_range);
  EXPECT_DOUBLE_EQ(0.55, seq[0]);
  EXPECT_DOUBLE_EQ(1.0, seq[1]);

  StepSequence seq2(0.5, false, true);
  // suport index [0, 1]
  // [0.5, 1.0]
  EXPECT_THROW(seq2[-1], std::out_of_range);
  EXPECT_THROW(seq2[2], std::out_of_range);
  EXPECT_DOUBLE_EQ(0.5, seq2[0]);
  EXPECT_DOUBLE_EQ(1.0, seq2[1]);
}

TEST(StepSequence, DefaultConstructorIncludesEndpoints)
{
  StepSequence seq(0.55);
  // suport index [0, 1, 2]
  // [0.0, 0.55, 1.0]
  EXPECT_THROW(seq[-1], std::out_of_range);
  EXPECT_DOUBLE_EQ(0.0, seq[0]);
  EXPECT_DOUBLE_EQ(0.55, seq[1]);
  EXPECT_DOUBLE_EQ(1.0, seq[2]);
}

TEST(StepSequence, ExcludesStartpoint)
{
  StepSequence seq(0.55, false, true);
  // suport index [0, 1]
  // [0.0, 0.55]
  EXPECT_THROW(seq[-1], std::out_of_range);
  EXPECT_THROW(seq[2], std::out_of_range);
  EXPECT_DOUBLE_EQ(0.55, seq[0]);

  StepSequence seq2(0.5, false, true, 0.0, 0.5);
  // support index [0]
  // [0.5]
  EXPECT_DOUBLE_EQ(0.5, seq2[0]);
  EXPECT_THROW(seq2[1], std::out_of_range);
}

TEST(StepSequence, ExcludesEndpoint)
{
  StepSequence seq(0.55, true, false);
  // suport index [0, 1]
  // [0.0, 0.55]
  EXPECT_THROW(seq[-1], std::out_of_range);
  EXPECT_THROW(seq[2], std::out_of_range);

  StepSequence seq2(0.5, true, false, 0.0, 1.0);
  // support index [0, 1]
  // [0.0, 0.5]
  EXPECT_DOUBLE_EQ(0.0, seq2[0]);
  EXPECT_DOUBLE_EQ(0.5, seq2[1]);
  EXPECT_THROW(seq[2], std::out_of_range);
}

TEST(StepSequence, ExcludesStartEndpoint)
{
  StepSequence seq(0.5, false, false);
  // support index [0]
  // [0.5]
  EXPECT_DOUBLE_EQ(0.5, seq[0]);
  EXPECT_THROW(seq[1], std::out_of_range);

  StepSequence seq2(0.5, false, false, 0.0, 0.5);
  EXPECT_THROW(seq2[0], std::out_of_range);
}

TEST(StepSequence, Iterator)
{
  StepSequence seq1(0.2, true, true);
  // [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
  auto itr1 = seq1.begin();
  EXPECT_DOUBLE_EQ(0, *itr1++);
  EXPECT_DOUBLE_EQ(0.2, *itr1++);
  EXPECT_DOUBLE_EQ(0.4, *itr1++);
  EXPECT_DOUBLE_EQ(0.6, *itr1++);
  EXPECT_DOUBLE_EQ(0.8, *itr1++);
  EXPECT_DOUBLE_EQ(1.0, *itr1++);
  EXPECT_DOUBLE_EQ(1.0, *itr1);

  StepSequence seq2(0.2, true, false);
  // [0.0, 0.2, 0.4, 0.6, 0.8]
  auto itr2 = seq2.begin();
  EXPECT_DOUBLE_EQ(0, *itr2++);
  EXPECT_DOUBLE_EQ(0.2, *itr2++);
  EXPECT_DOUBLE_EQ(0.4, *itr2++);
  EXPECT_DOUBLE_EQ(0.6, *itr2++);
  EXPECT_DOUBLE_EQ(0.8, *itr2++);
  EXPECT_DOUBLE_EQ(0.8, *itr2++);
  EXPECT_DOUBLE_EQ(0.8, *itr2);

  StepSequence seq3(-0.2, true, true, 1.0, 0.0);
  // [1.0 0.8 0.6 0.4 0.2 0.0]
  auto itr3 = seq3.begin();
  EXPECT_DOUBLE_EQ(1.0, *itr3++);
  EXPECT_DOUBLE_EQ(0.8, *itr3++);
  EXPECT_DOUBLE_EQ(0.6, *itr3++);
  EXPECT_DOUBLE_EQ(0.4, *itr3++);
  EXPECT_DOUBLE_EQ(0.2, *itr3++);
  EXPECT_DOUBLE_EQ(0.0, *itr3);
}

TEST(StepSequence, IteratorAlternateRange)
{
  StepSequence seq1(0.2, true, true, -1.0, 0.0);
  // [-1.0 -0.8 -0.6 -0.4 -0.2 0.0]
  auto itr1 = seq1.begin();
  EXPECT_DOUBLE_EQ(-1.0, *itr1++);
  EXPECT_DOUBLE_EQ(-0.8, *itr1++);
  EXPECT_DOUBLE_EQ(-0.6, *itr1++);
  EXPECT_DOUBLE_EQ(-0.4, *itr1++);
  EXPECT_DOUBLE_EQ(-0.2, *itr1++);
  EXPECT_DOUBLE_EQ(0.0, *itr1);

  StepSequence seq2(0.2, true, true, 24, 24.5);
  // [24, 24.2, 24.4, 24.5]
  auto itr2 = seq2.begin();
  EXPECT_DOUBLE_EQ(24, *itr2++);
  EXPECT_DOUBLE_EQ(24.2, *itr2++);
  EXPECT_DOUBLE_EQ(24.4, *itr2++);
  EXPECT_DOUBLE_EQ(24.5, *itr2++);
  EXPECT_DOUBLE_EQ(24.5, *itr2++);
  EXPECT_DOUBLE_EQ(24.5, *itr2);
}

TEST(StepSequence, IteratorStop)
{
  std::size_t count11 = 0;
  StepSequence seq11(0.3, true, true);
  double ref11[5] = {0.0, 0.3, 0.6, 0.9, 1.0};
  for (double v : seq11)
  {
    EXPECT_DOUBLE_EQ(ref11[count11], v);
    count11++;
  }
  EXPECT_EQ(5, count11);

  std::size_t count12 = 0;
  StepSequence seq12(0.3, true, false);
  double ref12[4] = {0.0, 0.3, 0.6, 0.9};
  for (double v : seq12)
  {
    EXPECT_DOUBLE_EQ(ref12[count12], v);
    count12++;
  }
  EXPECT_EQ(4, count12);

  std::size_t count13 = 0;
  StepSequence seq13(0.3, false, true);
  double ref13[4] = {0.3, 0.6, 0.9, 1.0};
  for (double v : seq13)
  {
    EXPECT_DOUBLE_EQ(ref13[count13], v);
    count13++;
  }
  EXPECT_EQ(4, count13);

  std::size_t count14 = 0;
  StepSequence seq14(0.3, false, false);
  double ref14[3] = {0.3, 0.6, 0.9};
  for (double v : seq14)
  {
    EXPECT_DOUBLE_EQ(ref14[count14], v);
    count14++;
  }
  EXPECT_EQ(3, count14);

  std::size_t count21 = 0;
  StepSequence seq21(0.2, true, true);
  double ref21[6] = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
  for (double v : seq21)
  {
    EXPECT_DOUBLE_EQ(ref21[count21], v);
    count21++;
  }
  EXPECT_EQ(6, count21);

  std::size_t count22 = 0;
  StepSequence seq22(0.2, true, false);
  double ref22[5] = {0.0, 0.2, 0.4, 0.6, 0.8};
  for (double v : seq22)
  {
    EXPECT_DOUBLE_EQ(ref22[count22], v);
    count22++;
  }
  EXPECT_EQ(5, count22);

  std::size_t count23 = 0;
  StepSequence seq23(0.2, false, true);
  double ref23[5] = {0.2, 0.4, 0.6, 0.8, 1.0};
  for (double v : seq23)
  {
    EXPECT_DOUBLE_EQ(ref23[count23], v);
    count23++;
  }
  EXPECT_EQ(5, count23);

  std::size_t count24 = 0;
  StepSequence seq24(0.2, false, false);
  double ref24[4] = {0.2, 0.4, 0.6, 0.8};
  for (double v : seq24)
  {
    EXPECT_DOUBLE_EQ(ref24[count24], v);
    count24++;
  }
  EXPECT_EQ(4, count24);
}

TEST(StepSequence, IteratorStopAlternateRange)
{
  std::size_t count1 = 0;
  StepSequence seq1(-0.2, true, true, 1.0, 0.0);
  double ref1[6] = {1.0, 0.8, 0.6, 0.4, 0.2, 0.0};
  for (double v : seq1)
    EXPECT_DOUBLE_EQ(ref1[count1++], v);
  EXPECT_EQ(6, count1);

  std::size_t count2 = 0;
  StepSequence seq2(0.3, true, true, 3, 3.7);
  double ref2[4] = {3.0, 3.3, 3.6, 3.7};
  for (double v : seq2)
    EXPECT_DOUBLE_EQ(ref2[count2++], v);
  EXPECT_EQ(4, count2);
}
