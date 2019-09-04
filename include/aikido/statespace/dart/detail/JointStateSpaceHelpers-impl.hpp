#include <memory>

#include "aikido/common/memory.hpp"

#include "aikido/common/metaprogramming.hpp"
#include "aikido/RnJoint.hpp"
#include "aikido/SE2Joint.hpp"
#include "aikido/SE3Joint.hpp"
#include "aikido/SO2Joint.hpp"
#include "aikido/SO3Joint.hpp"
#include "aikido/WeldJoint.hpp"

namespace aikido {
namespace statespace {
namespace dart {
namespace detail {

using Ptr = std::unique_ptr<JointStateSpace>;

//==============================================================================
template <class JointType>
struct createJointStateSpaceFor_impl
{
};

//==============================================================================
template <>
struct createJointStateSpaceFor_impl<const ::dart::dynamics::RevoluteJoint>
{
  static Ptr create(const ::dart::dynamics::RevoluteJoint* _joint)
  {
    if (_joint->isCyclic(0))
      return ::aikido::common::make_unique<SO2Joint>(_joint);
    else
      return ::aikido::common::make_unique<R1Joint>(_joint);
  }
};

//==============================================================================
template <>
struct createJointStateSpaceFor_impl<const ::dart::dynamics::PrismaticJoint>
{
  static Ptr create(const ::dart::dynamics::PrismaticJoint* _joint)
  {
    return ::aikido::common::make_unique<R1Joint>(_joint);
  }
};

//==============================================================================
template <>
struct createJointStateSpaceFor_impl<const ::dart::dynamics::TranslationalJoint>
{
  static Ptr create(const ::dart::dynamics::TranslationalJoint* _joint)
  {
    return ::aikido::common::make_unique<R3Joint>(_joint);
  }
};

//==============================================================================
template <>
struct createJointStateSpaceFor_impl<const ::dart::dynamics::BallJoint>
{
  static Ptr create(const ::dart::dynamics::BallJoint* _joint)
  {
    return ::aikido::common::make_unique<SO3Joint>(_joint);
  }
};

//==============================================================================
template <>
struct createJointStateSpaceFor_impl<const ::dart::dynamics::PlanarJoint>
{
  static Ptr create(const ::dart::dynamics::PlanarJoint* _joint)
  {
    return ::aikido::common::make_unique<SE2Joint>(_joint);
  }
};

//==============================================================================
template <>
struct createJointStateSpaceFor_impl<const ::dart::dynamics::FreeJoint>
{
  static Ptr create(const ::dart::dynamics::FreeJoint* _joint)
  {
    return ::aikido::common::make_unique<SE3Joint>(_joint);
  }
};

//==============================================================================
template <>
struct createJointStateSpaceFor_impl<const ::dart::dynamics::WeldJoint>
{
  static Ptr create(const ::dart::dynamics::WeldJoint* _joint)
  {
    return ::aikido::common::make_unique<WeldJoint>(_joint);
  }
};

//==============================================================================
using ConstSupportedJoints = common::type_list<
    const ::dart::dynamics::BallJoint,
    const ::dart::dynamics::FreeJoint,
    const ::dart::dynamics::PlanarJoint,
    const ::dart::dynamics::PrismaticJoint,
    const ::dart::dynamics::RevoluteJoint,
    const ::dart::dynamics::TranslationalJoint,
    const ::dart::dynamics::WeldJoint
    // TODO: Support ScrewJoint.
    // TODO: Support UniversalJoint.
    // TODO: Support EulerJoint.
    >;

} // namespace detail

//==============================================================================
template <class JointType>
std::unique_ptr<JointStateSpace> createJointStateSpaceFor(JointType* _joint)
{
  return detail::createJointStateSpaceFor_impl<JointType>::create(_joint);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
