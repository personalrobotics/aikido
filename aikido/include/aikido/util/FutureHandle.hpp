#ifndef AIKIDO_UTIL_FUTURE_HANDLE_
#define AIKIDO_UTIL_FUTURE_HANDLE_

#include <functional>

namespace aikido
{
namespace util
{
template <typename ResultType, typename FeedbackType>
class FutureHandle
{
public:
  FutureHandle() = default;
  FutureHandle(FutureHandle&& other) = default;
  FutureHandle& operator=(FutureHandle&& other) = default;

  /// uncopyable
  FutureHandle(const FutureHandle& other) = delete;
  /// uncopyable
  FutureHandle& operator=(const FutureHandle&) = delete;

  /// block and return operation result
  ResultType getResult();

  /// get latest feedback from operation
  FeedbackType getFeedback();  // optional?

  /// register a callback to handle new feedback
  void registerFeedbackHandler(
      std::function<void(FeedbackType)> callback);  // optional?

  /// non-blocking check if operation is complete and result is available
  bool isDone();

  /// block until operation is done (isDone() guaranteed to return true)
  void wait();

  /// cancel pending operation
  void cancel();
};
}
}

#endif  // AIKIDO_UTIL_FUTURE_HANDLE_
