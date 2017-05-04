#ifndef AIKIDO_RVIZ_SMARTPOINTERS_HPP_
#define AIKIDO_RVIZ_SMARTPOINTERS_HPP_

#include <memory>

// Define a typedef for const and non-const version of shared_ptr for the
// class X
#define AIKIDO_DEFINE_SHARED_PTR(X)                                            \
  class X;                                                                     \
  using X##Ptr = std::shared_ptr<X>;                                           \
  using X##ConstPtr = std::shared_ptr<const X>;

namespace aikido {
namespace rviz {

AIKIDO_DEFINE_SHARED_PTR(FrameMarker)
AIKIDO_DEFINE_SHARED_PTR(BodyNodeMarker)
AIKIDO_DEFINE_SHARED_PTR(SkeletonMarker)
AIKIDO_DEFINE_SHARED_PTR(TSRMarker)

} // namespace rviz
} // namespace aikido

#endif // ifndef AIKIDO_RVIZ_SMARTPOINTERS_HPP_
