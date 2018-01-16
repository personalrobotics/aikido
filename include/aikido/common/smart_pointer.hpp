#ifndef AIKIDO_COMMON_SMARTPOINTER_HPP_
#define AIKIDO_COMMON_SMARTPOINTER_HPP_

#include <dart/common/SmartPointer.hpp>

#define AIKIDO_COMMON_DECLARE_SMART_POINTERS(X)                                \
  DART_COMMON_MAKE_SHARED_WEAK(X)                                              \
  using Unique ## X ## Ptr      = std::unique_ptr< X >;                        \
  using UniqueConst ## X ## Ptr = std::unique_ptr< const X >;

namespace aikido {
namespace common {

// Declare here the smart pointers of aikido::common objects

} // namespace common
} // namespace aikido

#endif // AIKIDO_COMMON_SMARTPOINTER_HPP_
