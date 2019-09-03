#include <iostream>

#include <aikido/common/ExecutorThread.hpp>

namespace aikido {
namespace common {

//==============================================================================
ExecutorThread::~ExecutorThread()
{
  stop();
}

//==============================================================================
bool ExecutorThread::isRunning() const
{
  return mIsRunning.load();
}

//==============================================================================
void ExecutorThread::stop()
{
  mIsRunning.store(false);

  if (mThread.joinable())
    mThread.join();
}

//==============================================================================
void ExecutorThread::spin()
{
  auto currentTime = std::chrono::steady_clock::now();

  while (mIsRunning.load())
  {
    try
    {
      mCallback();
    }
    catch (const std::exception& e)
    {
      std::cerr << "Exception thrown by callback: " << e.what() << std::endl;
      // TODO: We should find another way to handle this error, so we don't
      // print directly to std::cerr. Unfortunately, we don't have a better
      // solution yet since Aikido doesn't use any particular logging framework.
      mIsRunning.store(false);

      break;
    }

    currentTime += mPeriod;
    std::this_thread::sleep_until(currentTime);
  }
}

} // namespace common
} // namespace aikido
