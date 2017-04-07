#ifndef AIKIDO_UTIL_EXECUTORMULTIPLEXER_HPP_
#define AIKIDO_UTIL_EXECUTORMULTIPLEXER_HPP_

#include <functional>
#include <mutex>
#include <vector>

namespace aikido {
namespace util {

/// Combine multiple executors (i.e. no argument callbacks) into one executor.
///
/// This helper class allows one ExecutorThread to call multiple executors by
/// sequentially calling the callbacks added to this class.
///
/// \sa ExecutorThread
class ExecutorMultiplexer final
{
public:
  /// Default constructor.
  ExecutorMultiplexer() = default;

  /// Default destructor.
  ~ExecutorMultiplexer() = default;

  /// Adds a callback. The added callbacks will be called by operator().
  ///
  /// The order of callback calling is implementation detail that is subject to
  /// change.
  ///
  /// \param[in] callback Any callable object that doesn't return and take any
  /// parameters.
  void addCallback(std::function<void()> callback);

  /// Removes all the added callbacks.
  void removeAllCallbacks();

  /// Returns true if no callback is added. Otherwise, returns false.
  bool isEmpty() const;

  /// Returns the number of added callbacks.
  std::size_t getNumCallbacks() const;

  /// Executes all the added callbacked in order of they added.
  void operator()();

private:
  /// Mutex for the list of callbacks. The array of callbacks will be locked
  /// during it's modified and the callbacks are called.
  std::mutex mMutex;

  /// Array of callbacks.
  std::vector<std::function<void()>> mCallbacks;
};

} // namespace util
} // namespace aikido

#endif // ifndef AIKIDO_UTIL_EXECUTORMULTIPLEXER_HPP_
