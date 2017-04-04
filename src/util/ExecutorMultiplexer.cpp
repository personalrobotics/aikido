#include <aikido/util/ExecutorMultiplexer.hpp>

namespace aikido {
namespace util {

//=============================================================================
void ExecutorMultiplexer::addCallback(std::function<void ()> _callback)
{
  std::lock_guard<std::mutex> lock{mMutex};
  mCallbacks.emplace_back(std::move(_callback));
}

//=============================================================================
void ExecutorMultiplexer::operator ()()
{
  std::lock_guard<std::mutex> lock{mMutex};

  for (auto& callback : mCallbacks)
    callback();
}

} // namespace util
} // namespace aikido
