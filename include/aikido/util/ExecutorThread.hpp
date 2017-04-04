#ifndef AIKIDO_UTIL_EXECUTORTHREAD_HPP_
#define AIKIDO_UTIL_EXECUTORTHREAD_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

namespace aikido {
namespace util {

/// ExecutorThread is a wrapper of std::thread that calls a callback
/// periodically.
///
/// \code
/// ExecutorThread exec(
///     []() { std::cout << "running\n";}, std::chrono::nanoseconds(1));
///
/// // thread is running
///
/// exec.stop();
/// \endcode
class ExecutorThread final
{
public:
  /// Constructs from callback and period. The thread begins execution
  /// immediately upon construction.
  /// \param[in] callback
  /// \param[in] period
  template <typename Duration>
  ExecutorThread(std::function<void ()> callback, const Duration& period);

  /// Default destructor. The thread is stopped as ExecutorThread is destructed.
  ~ExecutorThread();

  /// Returns true if the thread is running.
  bool isRunning() const;

  /// Stops the thread. It is safe to call this function even when the thread is
  /// already stopped.
  void stop();

private:
  /// The loop function that will be executed by the thread.
  void spin();

private:
  /// Callback to be periodically executed by the thread.
  std::function<void ()> mCallback;

  /// The callback is called in this period.
  std::chrono::milliseconds mPeriod;

  /// Flag whether the thread is running.
  std::atomic<bool> mIsRunning;

  /// Thread.
  std::thread mThread;
};

} // namespace util
} // namespace aikido

#include <aikido/util/detail/ExecutorThread-impl.hpp>

#endif // ifndef AIKIDO_UTIL_EXECUTORTHREAD_HPP_
