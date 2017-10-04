#ifndef AIKIDO_STATESPACE_DART_DETAIL_RNJOINTTRAITS_HPP_
#define AIKIDO_STATESPACE_DART_DETAIL_RNJOINTTRAITS_HPP_

#include <dart/dart.hpp>

namespace aikido {
namespace statespace {
namespace dart {
namespace detail {

//==============================================================================
template <int N>
struct RJointTraits
{
  using DartJoint
      = ::dart::dynamics::GenericJoint<::dart::math::RealVectorSpace<N>>;
};

//==============================================================================
template <>
struct RJointTraits<0>
{
  using DartJoint = ::dart::dynamics::WeldJoint;
};

} // namespace detail
} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // AIKIDO_STATESPACE_DART_DETAIL_RNJOINTTRAITS_HPP_
