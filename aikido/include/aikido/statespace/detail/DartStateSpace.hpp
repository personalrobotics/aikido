#include <dart/common/StlHelpers.h>
#include <aikido/statespace/RealVectorJointStateSpace.hpp>
#include <aikido/statespace/SO2JointStateSpace.hpp>
#include <aikido/statespace/SE3JointStateSpace.hpp>

namespace aikido {
namespace statespace {
namespace detail {

using dart::common::make_unique;
using Ptr = std::unique_ptr<JointStateSpace>;

//=============================================================================
template <class JointType>
struct createJointStateSpaceFor_impl {};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<dart::dynamics::RevoluteJoint>
{
  static Ptr create(dart::dynamics::RevoluteJoint* _joint)
  {
    if (_joint->isCyclic(0))
       return make_unique<SO2JointStateSpace>(_joint);
    else
       return make_unique<RealVectorJointStateSpace>(_joint);
  }
};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<dart::dynamics::PrismaticJoint>
{
  static Ptr create(dart::dynamics::PrismaticJoint* _joint)
  {
    return make_unique<RealVectorJointStateSpace>(_joint);
  }
};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<dart::dynamics::TranslationalJoint>
{
  static Ptr create(dart::dynamics::TranslationalJoint* _joint)
  {
    return make_unique<RealVectorJointStateSpace>(_joint);
  }
};

//=============================================================================
template <>
struct createJointStateSpaceFor_impl<dart::dynamics::FreeJoint>
{
  static Ptr create(dart::dynamics::FreeJoint* _joint)
  {
    return make_unique<SE3JointStateSpace>(_joint);
  }
};

//=============================================================================
template <class... Args>
struct ForOneOf {};

template <>
struct ForOneOf<>
{
  static Ptr create(dart::dynamics::Joint* _joint)
  {
    return nullptr;
  }
};

template <class Arg, class... Args>
struct ForOneOf<Arg, Args...>
{
  static Ptr create(dart::dynamics::Joint* _joint)
  {
    if (&_joint->getType() == &Arg::getStaticType())
    {
      return createJointStateSpaceFor_impl<Arg>::create(
        static_cast<Arg*>(_joint));
    }
    else
    {
      return ForOneOf<Args...>::create(_joint);
    }
  }
};


} // namespace detail

//=============================================================================
template <class JointType>
std::unique_ptr<JointStateSpace> createJointStateSpaceFor(JointType* _joint)
{
  return detail::createJointStateSpaceFor_impl<JointType>::create(_joint);
}

//=============================================================================
template <class Space>
const StateSpace* MetaSkeletonStateSpace::getJointSpace(
  const dart::dynamics::Joint* _joint) const
{
  const auto index = mMetaSkeleton->getIndexOf(_joint, true);
  if (index == dart::dynamics::INVALID_INDEX)
    return nullptr;

  return getSubSpace(index);
}

} // namespace statespace
} // namespace aikido
