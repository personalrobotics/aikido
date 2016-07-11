#ifndef AIKIDO_UTIL_EXECUTORTHREAD_HPP_
#define AIKIDO_UTIL_EXECUTORTHREAD_HPP_
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

namespace aikido {
namespace util {

class ExecutorThread final
{
public:
  ExecutorThread(
    std::function<void ()> _callback,
    std::chrono::milliseconds _period);

  ~ExecutorThread();

  bool is_running() const;

  void stop();

private:
  void spin();

  std::function<void ()> mCallback;
  std::chrono::milliseconds mPeriod;
  std::atomic<bool> mIsRunning;
  std::thread mThread;
};

} // namespace util
} // namespace aikido

#endif // ifndef AIKIDO_UTIL_EXECUTORTHREAD_HPP_
