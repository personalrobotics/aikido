#include <aikido/util/ExecutorMultiplexer.hpp>

#include <dart/dart.hpp>

namespace aikido {
namespace util {

//=============================================================================
void ExecutorMultiplexer::addCallback(std::function<void ()> callback)
{
  std::lock_guard<std::mutex> lock{mMutex};
  DART_UNUSED(lock);

  mCallbacks.emplace_back(std::move(callback));
}

//=============================================================================
void ExecutorMultiplexer::removeAllCallbacks()
{
  mCallbacks.clear();
}

//=============================================================================
bool ExecutorMultiplexer::isEmpty() const
{
  return mCallbacks.empty();
}

//=============================================================================
std::size_t ExecutorMultiplexer::getNumCallbacks() const
{
  return mCallbacks.size();
}

//=============================================================================
void ExecutorMultiplexer::operator()()
{
  std::lock_guard<std::mutex> lock{mMutex};
  DART_UNUSED(lock);

  for (const auto& callback : mCallbacks)
    callback();
}

} // namespace util
} // namespace aikido
