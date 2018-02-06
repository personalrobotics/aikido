#ifndef AIKIDO_COMMON_POINTERS_HPP_
#define AIKIDO_COMMON_POINTERS_HPP_

#include <dart/common/SmartPointer.hpp>

// clang-format off

// This macro must be used in the class' namespace. For example,
//
//   namespace aikido {
//   namespace constraint {
//
//   AIKIDO_DECLARE_POINTERS(Testable)
//
//   class Testable
//   { ... };
//   
//   } // namespace constraint
//   } // namespace aikido
//  
#define AIKIDO_DECLARE_POINTERS(X)                                             \
  DART_COMMON_MAKE_SHARED_WEAK(X)                                              \
  using Unique ## X ## Ptr      = std::unique_ptr< X >;                        \
  using UniqueConst ## X ## Ptr = std::unique_ptr< const X >;

// clang-format on

#endif // AIKIDO_COMMON_POINTERS_HPP_
