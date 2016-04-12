#include <sstream>
#include <dart/common/StlHelpers.h>
#include "../../statespace/dart/RealVectorJointStateSpace.hpp"
#include "../../statespace/dart/SO2JointStateSpace.hpp"
#include "../../statespace/dart/SO3JointStateSpace.hpp"
#include "../../statespace/dart/SE2JointStateSpace.hpp"
#include "../../statespace/dart/SE3JointStateSpace.hpp"
#include "../../util/metaprogramming.hpp"
#include "../uniform/RealVectorBoxConstraint.hpp"
#include "../SatisfiedConstraint.hpp"

namespace aikido {
namespace constraint {
namespace detail {

//=============================================================================
using JointStateSpaceTypeList = util::type_list<
  statespace::RealVectorJointStateSpace,
  statespace::SO2JointStateSpace,
  statespace::SO3JointStateSpace,
  statespace::SE2JointStateSpace,
  statespace::SE3JointStateSpace
>;

//=============================================================================
template <template <class> class Factory, class BaseParameter, class TypeList>
struct ForOneOf {};

template <template <class> class Factory, class BaseParameter>
struct ForOneOf<Factory, BaseParameter, util::type_list<>>
{
  static std::nullptr_t create(std::shared_ptr<BaseParameter> /* unused */)
  {
    return nullptr;
  }
};

template <template <class> class Factory, class BaseParameter,
          class Arg, class... Args>
struct ForOneOf<Factory, BaseParameter, util::type_list<Arg, Args...>>
{
  static auto create(std::shared_ptr<BaseParameter> _delegate)
    -> decltype(Factory<Arg>::create(
          std::dynamic_pointer_cast<Arg>(_delegate)))
  {
    if (auto arg = std::dynamic_pointer_cast<Arg>(_delegate))
    {
      return Factory<Arg>::create(std::move(arg));
    }
    else
    {
      return ForOneOf<Factory, BaseParameter, util::type_list<Args...>>
        ::create(std::move(_delegate));
    }
  }
};

//=============================================================================
bool isLimited(const dart::dynamics::Joint* _joint)
{
  for (size_t i = 0; i < _joint->getNumDofs(); ++i)
  {
    if (_joint->hasPositionLimit(i))
      return true;
  }
  return false;
}

//=============================================================================
template <class OutputConstraint>
std::unique_ptr<OutputConstraint> createBoxConstraint(
  std::shared_ptr<statespace::RealVectorJointStateSpace> _stateSpace,
  std::unique_ptr<util::RNG> _rng)
{
  const auto joint = _stateSpace->getJoint();

  if (isLimited(joint))
  {
    const size_t dimension = joint->getNumDofs();
    Eigen::VectorXd lowerBounds(dimension);
    Eigen::VectorXd upperBounds(dimension);

    for (size_t i = 0; i < dimension; ++i)
    {
      lowerBounds[i] = joint->getPositionLowerLimit(i);
      upperBounds[i] = joint->getPositionUpperLimit(i);
    }

    return dart::common::make_unique<statespace::RealVectorBoxConstraint>(
      std::move(_stateSpace), std::move(_rng), lowerBounds, upperBounds);
  }
  else
  {
    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
}

//=============================================================================
template <class T>
struct createDifferentiableFor_impl { };

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::RealVectorJointStateSpace>
{
  using StateSpace = statespace::RealVectorJointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Differentiable>(std::move(_stateSpace), nullptr);
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::SO2JointStateSpace>
{
  using StateSpace = statespace::SO2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::SO3JointStateSpace>
{
  using StateSpace = statespace::SO3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::SE2JointStateSpace>
{
  using StateSpace = statespace::SE2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No DifferentiableConstraint is available for SE2JointStateSpace.");
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::SE3JointStateSpace>
{
  using StateSpace = statespace::SE3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No DifferentiableConstraint is available for SE3JointStateSpace.");
  }
};

//=============================================================================
template <class T>
struct createTestableFor_impl { };

//=============================================================================
template <>
struct createTestableFor_impl<statespace::RealVectorJointStateSpace>
{
  using StateSpace = statespace::RealVectorJointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<TestableConstraint>(
      std::move(_stateSpace), nullptr);
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::SO2JointStateSpace>
{
  using StateSpace = statespace::SO2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::SO3JointStateSpace>
{
  using StateSpace = statespace::SO3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::SE2JointStateSpace>
{
  using StateSpace = statespace::SE2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No TestableConstraint is available for SE2JointStateSpace.");
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::SE3JointStateSpace>
{
  using StateSpace = statespace::SE3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No TestableConstraint is available for SE3JointStateSpace.");
  }
};

} // namespace detail
} // namespace constraint
} // namespace aikido
