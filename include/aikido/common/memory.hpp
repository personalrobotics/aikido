#ifndef AIKIDO_COMMON_MEMORY_HPP_
#define AIKIDO_COMMON_MEMORY_HPP_

#include <memory>

namespace aikido {
namespace common {

template <typename T, typename... Args>
::std::unique_ptr<T> make_unique(Args&&... args);

} // namespace common
} // namespace aikido

#include "aikido/common/detail/memory-impl.hpp"

#endif // AIKIDO_COMMON_MEMORY_HPP_
