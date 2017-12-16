#ifndef AIKIDO_COMMON_SMARTPOINTER_HPP_
#define AIKIDO_COMMON_SMARTPOINTER_HPP_

#include <memory>

// Define a typedef for const and non-const version of shared_ptr, weak_ptr,
// unique_ptr for the class X
#define AIKIDO_COMMON_DECLARE_SMART_POINTER(X)                                 \
  class X;                                                                     \
  using X##Ptr = std::shared_ptr<X>;                                           \
  using Const##X##Ptr = std::shared_ptr<const X>;                              \
  using Weak##X##Ptr = std::weak_ptr<X>;                                       \
  using WeakConst##X##Ptr = std::weak_ptr<const X>;                            \
  using Unique##X##Ptr = std::unique_ptr<X>;                                   \
  using UniqueConst##X##Ptr = std::unique_ptr<const X>;

#endif // AIKIDO_COMMON_SMARTPOINTER_HPP_
