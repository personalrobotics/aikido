#include <aikido/common/ExecutorMultiplexer.hpp>
#include <aikido/common/ExecutorThread.hpp>
#include <gtest/gtest.h>

using namespace aikido::common;

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

//==============================================================================
TEST(ExecutorThread, ExceptionThrownByCallback)
{
  ExecutorThread exec(
      []() { throw std::exception(); }, std::chrono::milliseconds(1));

  // Assumed that the 3 second sleep is essentially a timeout for the test. If
  // this is not the case, either increase the sleep time or remove this test
  // (unless there is a better test for this).
  std::this_thread::sleep_for(std::chrono::seconds(3));

  EXPECT_TRUE(!exec.isRunning());
}
