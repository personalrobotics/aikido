#ifndef AIKIDO_COMMON_DETAIL_MEMORY_IMPL_HPP_
#define AIKIDO_COMMON_DETAIL_MEMORY_IMPL_HPP_

#include "aikido/common/memory.hpp"

namespace aikido {
namespace common {

//==============================================================================
template <typename T, typename... Args>
::std::unique_ptr<T> make_unique(Args&&... args)
{
#if __cplusplus < 201300
  return ::std::unique_ptr<T>(new T(::std::forward<Args>(args)...));
#else
  return ::std::make_unique<T>(::std::forward<Args>(args)...);
#endif
}

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_DETAIL_MEMORY_IMPL_HPP_
