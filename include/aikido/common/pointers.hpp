#ifndef AIKIDO_COMMON_POINTERS_HPP_
#define AIKIDO_COMMON_POINTERS_HPP_

#include <memory>

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
  class X ;                                                                    \
  using X ## Ptr                = std::shared_ptr< X >;                        \
  using Const ## X ## Ptr       = std::shared_ptr< const X >;                  \
  using Weak ## X ## Ptr        = std::weak_ptr< X >;                          \
  using WeakConst ## X ## Ptr   = std::weak_ptr< const X >;                    \
  using Unique ## X ## Ptr      = std::unique_ptr< X >;                        \
  using UniqueConst ## X ## Ptr = std::unique_ptr< const X >;

// clang-format on

#endif // AIKIDO_COMMON_POINTERS_HPP_
