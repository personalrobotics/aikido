#include <aikido/util/ExecutorThread.hpp>

namespace aikido {
namespace util {

//==============================================================================
template <typename Duration>
ExecutorThread::ExecutorThread(
    std::function<void ()> callback, const Duration& period)
  : mCallback{std::move(callback)},
    mPeriod{std::chrono::duration_cast<std::chrono::milliseconds>(period)},
    mIsRunning{true},
    mThread{std::thread{&ExecutorThread::spin, this}}
{
  // Do nothing
}

} // namespace util
} // namespace aikido
