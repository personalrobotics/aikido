#ifndef AIKIDO_COMMON_UTIL_HPP_
#define AIKIDO_COMMON_UTIL_HPP_

#include <cmath>
#include <future>

namespace aikido {
namespace common {

/// Make a pre-made exceptional std::future
template <typename T>
inline std::future<T> make_exceptional_future(std::string error_message)
{
  auto promise = std::promise<T>();
  promise.set_exception(
      std::make_exception_ptr(std::runtime_error(error_message)));
  return promise.get_future();
}

/// Make a pre-made ready std::future
template <typename T>
inline std::future<T> make_ready_future(T obj)
{
  auto promise = std::promise<T>();
  promise.set_value(obj);
  return promise.get_future();
}

/// Check for a value near-zero
#ifndef AIKIDO_COMMON_NEARZERO
#define AIKIDO_COMMON_NEARZERO 1E-8
#endif

inline bool FuzzyZero(double value, double tol = AIKIDO_COMMON_NEARZERO)
{
  return (abs(value) <= tol);
}

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_UTIL_HPP_
