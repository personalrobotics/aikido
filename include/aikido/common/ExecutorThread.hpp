#ifndef AIKIDO_COMMON_EXECUTORTHREAD_HPP_
#define AIKIDO_COMMON_EXECUTORTHREAD_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

namespace aikido {
namespace common {

/// ExecutorThread is a wrapper of std::thread that calls a callback
/// periodically.
///
/// If you want to let ExecutorThread calls multiple callbacks then consider
/// using ExecutorMultiplexer.
///
/// \code
/// ExecutorThread exec(
///     []() { std::cout << "running...\n"; }, std::chrono::milliseconds(10));
///
/// // thread is running
///
/// // The destructor of ExecutorThread stops the thread.
/// \endcode
///
/// \sa ExecutorMultiplexer
class ExecutorThread final
{
public:
  /// Constructs from callback and period. The thread begins execution
  /// immediately upon construction.
  /// \param[in] callback Callback to be repeatedly executed by the thread.
  /// \param[in] period The period of calling the callback.
  template <typename Duration>
  ExecutorThread(std::function<void()> callback, const Duration& period);

  /// Default destructor. The thread stops as ExecutorThread is destructed.
  ~ExecutorThread();

  /// Returns true if the thread is running.
  bool isRunning() const;

  /// Stops the thread. It is safe to call this function even when the thread
  /// already stopped.
  void stop();

private:
  /// The loop function that will be executed by the thread.
  void spin();

private:
  /// Callback to be periodically executed by the thread.
  std::function<void()> mCallback;

  /// The callback is called in this period.
  std::chrono::milliseconds mPeriod;

  /// Flag whether the thread is running.
  std::atomic<bool> mIsRunning;

  /// Thread.
  std::thread mThread;
};

} // namespace common
} // namespace aikido

#include "aikido/common/detail/ExecutorThread-impl.hpp"

#endif // AIKIDO_COMMON_EXECUTORTHREAD_HPP_
