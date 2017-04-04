#include <aikido/util/ExecutorMultiplexer.hpp>
#include <aikido/util/ExecutorThread.hpp>
#include <gtest/gtest.h>

using namespace aikido::util;

static int numCalled = 0;

void foo() { numCalled++; }

//==============================================================================
TEST(ExecutorMultiplexer, Execute)
{
  ExecutorMultiplexer exec;
  EXPECT_TRUE(exec.isEmpty());
  EXPECT_TRUE(exec.getNumCallbacks() == 0u);

  exec.addCallback(foo);
  exec.addCallback([]() {});
  EXPECT_TRUE(!exec.isEmpty());
  EXPECT_TRUE(exec.getNumCallbacks() == 2u);

  EXPECT_TRUE(numCalled == 0);
  exec();
  EXPECT_TRUE(numCalled == 1);

  exec.removeAllCallbacks();
  EXPECT_TRUE(exec.isEmpty());

  EXPECT_TRUE(numCalled == 1);
  exec();
  EXPECT_TRUE(numCalled == 1);
}

//==============================================================================
TEST(ExecutorThread, Execute)
{
  ExecutorThread exec([]() {}, std::chrono::nanoseconds(1));

  EXPECT_TRUE(exec.isRunning());
  exec.stop();
  EXPECT_TRUE(!exec.isRunning());
}
