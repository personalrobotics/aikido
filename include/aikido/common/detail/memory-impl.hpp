#ifndef AIKIDO_COMMON_DETAIL_MEMORY_IMPL_HPP_
#define AIKIDO_COMMON_DETAIL_MEMORY_IMPL_HPP_

#include "aikido/common/memory.hpp"

namespace aikido {
namespace common {

//==============================================================================
template<typename T, typename... Args>
::std::unique_ptr<T> make_unique(Args&&... args)
{
  return ::std::unique_ptr<T>(new T(::std::forward<Args>(args)...));
}
// TODO(JS): This is a stopgap solution as it was omitted from C++11 as "partly
// an oversight". This can be replaced by std::make_unique<T> of the standard
// library when we migrate to using C++14.

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_DETAIL_MEMORY_IMPL_HPP_
