#include "aikido/common/ExecutorThread.hpp"

namespace aikido {
namespace common {

//==============================================================================
template <typename Duration>
ExecutorThread::ExecutorThread(
    std::function<void()> callback, const Duration& period)
  : mCallback{std::move(callback)}
  , mPeriod{std::chrono::duration_cast<std::chrono::milliseconds>(period)}
  , mIsRunning{true}
  , mThread{std::thread{&ExecutorThread::spin, this}}
{
  // Do nothing
}

} // namespace common
} // namespace aikido
