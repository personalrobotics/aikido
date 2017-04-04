#include <iostream>
#include <aikido/util/ExecutorThread.hpp>

namespace aikido {
namespace util {

//=============================================================================
ExecutorThread::ExecutorThread(
      std::function<void ()> _callback,
      std::chrono::milliseconds _period)
  : mCallback{std::move(_callback)}
  , mPeriod{_period}
  , mIsRunning{true}
{
  mThread = std::thread(&ExecutorThread::spin, this);
}

//=============================================================================
ExecutorThread::~ExecutorThread()
{
  stop();
}

//=============================================================================
bool ExecutorThread::is_running() const
{
  return mIsRunning.load();
}

//=============================================================================
void ExecutorThread::stop()
{
  mIsRunning.store(false);
  mThread.join();
}

//=============================================================================
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
      mIsRunning.store(false);
    }

    currentTime += mPeriod;
    std::this_thread::sleep_until(currentTime);
  }
}

} // namespace util
} // namespace aikido
