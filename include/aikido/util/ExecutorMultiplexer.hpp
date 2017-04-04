#ifndef AIKIDO_UTIL_EXECUTORMULTIPLEXER_HPP_
#define AIKIDO_UTIL_EXECUTORMULTIPLEXER_HPP_
#include <functional>
#include <mutex>
#include <vector>

namespace aikido {
namespace util {

class ExecutorMultiplexer final
{
public:
  ExecutorMultiplexer() = default;
  ~ExecutorMultiplexer() = default;

  void addCallback(std::function<void ()> _callback);

  void operator ()();

private:
  std::mutex mMutex;
  std::vector<std::function<void ()>> mCallbacks;
};

} // namespace util
} // namespace aikido

#endif // ifndef AIKIDO_UTIL_EXECUTORMULTIPLEXER_HPP_
