#include <memory>
#include <dart/common/StlHelpers.h>
#include "../RnJoint.hpp"
#include "../SO2JointStateSpace.hpp"
#include "../SO3JointStateSpace.hpp"
#include "../SE2JointStateSpace.hpp"
#include "../SE3JointStateSpace.hpp"
#include "../../../util/metaprogramming.hpp"

namespace aikido {
namespace statespace {
namespace dart {
namespace detail {

using ::dart::common::make_unique;
using Ptr = std::unique_ptr<JointStateSpace>;

//=============================================================================
template <class JointType>
struct createJointStateSpaceFor_impl {};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<::dart::dynamics::RevoluteJoint>
{
  static Ptr create(::dart::dynamics::RevoluteJoint* _joint)
  {
    if (_joint->isCyclic(0))
       return make_unique<SO2JointStateSpace>(_joint);
    else
       return make_unique<RnJoint>(_joint);
  }
};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<::dart::dynamics::PrismaticJoint>
{
  static Ptr create(::dart::dynamics::PrismaticJoint* _joint)
  {
    return make_unique<RnJoint>(_joint);
  }
};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<::dart::dynamics::TranslationalJoint>
{
  static Ptr create(::dart::dynamics::TranslationalJoint* _joint)
  {
    return make_unique<RnJoint>(_joint);
  }
};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<::dart::dynamics::BallJoint>
{
  static Ptr create(::dart::dynamics::BallJoint* _joint)
  {
    return make_unique<SO3JointStateSpace>(_joint);
  }
};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<::dart::dynamics::PlanarJoint>
{
  static Ptr create(::dart::dynamics::PlanarJoint* _joint)
  {
    return make_unique<SE2JointStateSpace>(_joint);
  }
};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<::dart::dynamics::FreeJoint>
{
  static Ptr create(::dart::dynamics::FreeJoint* _joint)
  {
    return make_unique<SE3JointStateSpace>(_joint);
  }
};

//=============================================================================
using SupportedJoints = util::type_list<
  ::dart::dynamics::BallJoint,
  ::dart::dynamics::FreeJoint,
  ::dart::dynamics::PlanarJoint,
  ::dart::dynamics::PrismaticJoint,
  ::dart::dynamics::RevoluteJoint,
  ::dart::dynamics::TranslationalJoint
  // TODO: Support ScrewJoint.
  // TODO: Support WeldJoint.
  // TODO: Support UniversalJoint.
  // TODO: Support EulerJoint.
>;

} // namespace detail

//=============================================================================
template <class JointType>
std::unique_ptr<JointStateSpace> createJointStateSpaceFor(JointType* _joint)
{
  return detail::createJointStateSpaceFor_impl<JointType>::create(_joint);
}

} // namespace dart
} // namespace statespace
} // namespace aikido
