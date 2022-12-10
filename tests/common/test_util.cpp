#include <gtest/gtest.h>

#include <aikido/common/util.hpp>

using namespace aikido;
using aikido::common::FuzzyZero;
using aikido::common::make_exceptional_future;
using aikido::common::make_ready_future;

//==============================================================================
TEST(Util, Futures)
{
  std::future<int> exceptionalFuture = make_exceptional_future<int>("Testing");
  EXPECT_THROW(exceptionalFuture.get(), std::runtime_error);

  std::future<int> readyFuture = make_ready_future(5);
  EXPECT_EQ(readyFuture.get(), 5);
}

//==============================================================================
TEST(Util, FuzzyZero)
{
  EXPECT_EQ(FuzzyZero(1E-9), true);
  EXPECT_EQ(FuzzyZero(1E-7), false);
}
