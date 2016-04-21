#include <sstream>
#include <dart/common/StlHelpers.h>
#include "../../statespace/dart/RealVectorJointStateSpace.hpp"
#include "../../statespace/dart/SO2JointStateSpace.hpp"
#include "../../statespace/dart/SO3JointStateSpace.hpp"
#include "../../statespace/dart/SE2JointStateSpace.hpp"
#include "../../statespace/dart/SE3JointStateSpace.hpp"
#include "../../util/metaprogramming.hpp"
#include "../uniform/RealVectorBoxConstraint.hpp"
#include "../uniform/SO2UniformSampler.hpp"
#include "../uniform/SO3UniformSampler.hpp"
#include "../SatisfiedConstraint.hpp"

namespace aikido {
namespace constraint {
namespace detail {

//=============================================================================
template <template <class> class Factory, class BaseParameter, class TypeList>
struct ForOneOf {};

template <template <class> class Factory, class BaseParameter>
struct ForOneOf<Factory, BaseParameter, util::type_list<>>
{
  template <class... Parameters>
  static std::nullptr_t create(std::shared_ptr<BaseParameter> /* unused */,
                               Parameters&&... /* unused */)
  {
    return nullptr;
  }
};

template <template <class> class Factory, class BaseParameter,
          class Arg, class... Args>
struct ForOneOf<Factory, BaseParameter, util::type_list<Arg, Args...>>
{
  template <class... Parameters>
  static auto create(std::shared_ptr<BaseParameter> _delegate,
                     Parameters&&... _params)
    -> decltype(Factory<Arg>::create(
         std::dynamic_pointer_cast<Arg>(_delegate),
         std::forward<Parameters>(_params)...))
  {
    if (auto arg = std::dynamic_pointer_cast<Arg>(_delegate))
      return Factory<Arg>::create(
        std::move(arg), std::forward<Parameters>(_params)...);
    else
      return ForOneOf<
        Factory, BaseParameter, util::type_list<Args...>>::create(
          std::move(_delegate), std::forward<Parameters>(_params)...);
  }
};

//=============================================================================
inline bool isLimited(const dart::dynamics::Joint* _joint)
{
  for (size_t i = 0; i < _joint->getNumDofs(); ++i)
  {
    if (_joint->hasPositionLimit(i))
      return true;
  }
  return false;
}

inline Eigen::VectorXd getPositionLowerLimits(
  const dart::dynamics::Joint* _joint)
{
  const auto dimension = _joint->getNumDofs();
  Eigen::VectorXd bounds(dimension);

  for (size_t i = 0; i < dimension; ++i)
    bounds[i] = _joint->getPositionLowerLimit(i);

  return bounds;
}

inline Eigen::VectorXd getPositionUpperLimits(
  const dart::dynamics::Joint* _joint)
{
  const auto dimension = _joint->getNumDofs();
  Eigen::VectorXd bounds(dimension);

  for (size_t i = 0; i < dimension; ++i)
    bounds[i] = _joint->getPositionUpperLimit(i);

  return bounds;
}

//=============================================================================
using JointStateSpaceTypeList = util::type_list<
  statespace::dart::RealVectorJointStateSpace,
  statespace::dart::SO2JointStateSpace,
  statespace::dart::SO3JointStateSpace,
  statespace::dart::SE2JointStateSpace,
  statespace::dart::SE3JointStateSpace
>;

template <class T>
struct createDifferentiableFor_impl { };

template <class T>
struct createTestableFor_impl { };

template <class T>
struct createProjectableFor_impl { };

template <class T>
struct createSampleableFor_impl { };

//=============================================================================
template <class OutputConstraint>
std::unique_ptr<OutputConstraint> createBoxConstraint(
  std::shared_ptr<statespace::dart::RealVectorJointStateSpace> _stateSpace,
  std::unique_ptr<util::RNG> _rng)
{
  const auto joint = _stateSpace->getJoint();

  if (isLimited(joint))
    return dart::common::make_unique<statespace::RealVectorBoxConstraint>(
      std::move(_stateSpace), std::move(_rng),
      getPositionLowerLimits(joint), getPositionUpperLimits(joint));
  else
    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
}

template <>
struct createDifferentiableFor_impl<statespace::dart::RealVectorJointStateSpace>
{
  using StateSpace = statespace::dart::RealVectorJointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Differentiable>(std::move(_stateSpace), nullptr);
  }
};

template <>
struct createTestableFor_impl<statespace::dart::RealVectorJointStateSpace>
{
  using StateSpace = statespace::dart::RealVectorJointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<TestableConstraint>(
      std::move(_stateSpace), nullptr);
  }
};

template <>
struct createProjectableFor_impl<statespace::dart::RealVectorJointStateSpace>
{
  using StateSpace = statespace::dart::RealVectorJointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Projectable>(
      std::move(_stateSpace), nullptr);
  }
};

template <>
struct createSampleableFor_impl<statespace::dart::RealVectorJointStateSpace>
{
  using StateSpace = statespace::dart::RealVectorJointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<SampleableConstraint> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    const auto joint = _stateSpace->getJoint();

    if (isLimited(joint))
      return dart::common::make_unique<statespace::RealVectorBoxConstraint>(
        std::move(_stateSpace), std::move(_rng),
        getPositionLowerLimits(joint), getPositionUpperLimits(joint));
    else
      throw std::runtime_error(
        "Unable to create Sampleable for unbounded RealVectorStateSpace.");
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SO2JointStateSpace>
{
  using StateSpace = statespace::dart::SO2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

template <>
struct createTestableFor_impl<statespace::dart::SO2JointStateSpace>
{
  using StateSpace = statespace::dart::SO2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

template <>
struct createProjectableFor_impl<statespace::dart::SO2JointStateSpace>
{
  using StateSpace = statespace::dart::SO2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

template <>
struct createSampleableFor_impl<statespace::dart::SO2JointStateSpace>
{
  using StateSpace = statespace::dart::SO2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<SampleableConstraint> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2JointStateSpace must not have limits.");

    return dart::common::make_unique<
      statespace::SO2StateSpaceSampleableConstraint>(
        std::move(_stateSpace), std::move(_rng));
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SO3JointStateSpace>
{
  using StateSpace = statespace::dart::SO3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

template <>
struct createTestableFor_impl<statespace::dart::SO3JointStateSpace>
{
  using StateSpace = statespace::dart::SO3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

template <>
struct createProjectableFor_impl<statespace::dart::SO3JointStateSpace>
{
  using StateSpace = statespace::dart::SO3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3JointStateSpace must not have limits.");

    return dart::common::make_unique<SatisfiedConstraint>(
      std::move(_stateSpace));
  }
};

template <>
struct createSampleableFor_impl<statespace::dart::SO3JointStateSpace>
{
  using StateSpace = statespace::dart::SO3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<SampleableConstraint> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3JointStateSpace must not have limits.");

    return dart::common::make_unique<statespace::SO3UniformSampler>(
      std::move(_stateSpace), std::move(_rng));
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SE2JointStateSpace>
{
  using StateSpace = statespace::dart::SE2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No DifferentiableConstraint is available for SE2JointStateSpace.");
  }
};

template <>
struct createTestableFor_impl<statespace::dart::SE2JointStateSpace>
{
  using StateSpace = statespace::dart::SE2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No TestableConstraint is available for SE2JointStateSpace.");
  }
};

template <>
struct createProjectableFor_impl<statespace::dart::SE2JointStateSpace>
{
  using StateSpace = statespace::dart::SE2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No Projectable is available for SE2JointStateSpace.");
  }
};

template <>
struct createSampleableFor_impl<statespace::dart::SE2JointStateSpace>
{
  using StateSpace = statespace::dart::SE2JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<SampleableConstraint> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    throw std::runtime_error(
      "No Sampleable is available for SE2JointStateSpace.");
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SE3JointStateSpace>
{
  using StateSpace = statespace::dart::SE3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No DifferentiableConstraint is available for SE3JointStateSpace.");
  }
};

template <>
struct createTestableFor_impl<statespace::dart::SE3JointStateSpace>
{
  using StateSpace = statespace::dart::SE3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<TestableConstraint> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No TestableConstraint is available for SE3JointStateSpace.");
  }
};

template <>
struct createProjectableFor_impl<statespace::dart::SE3JointStateSpace>
{
  using StateSpace = statespace::dart::SE3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No Projectable is available for SE3JointStateSpace.");
  }
};

template <>
struct createSampleableFor_impl<statespace::dart::SE3JointStateSpace>
{
  using StateSpace = statespace::dart::SE3JointStateSpace;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<SampleableConstraint> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    throw std::runtime_error(
      "No Sampleable is available for SE3JointStateSpace.");
  }
};

} // namespace detail

//=============================================================================
template <class Space>
std::unique_ptr<Differentiable> createDifferentiableBoundsFor(
  std::shared_ptr<Space> _stateSpace)
{
  return detail::createDifferentiableFor_impl<Space>::create(
    std::move(_stateSpace));
}

//=============================================================================
template <class Space>
std::unique_ptr<Projectable> createProjectableBoundsFor(
  std::shared_ptr<Space> _stateSpace)
{
  return detail::createProjectableFor_impl<Space>::create(
    std::move(_stateSpace));
}

//=============================================================================
template <class Space>
std::unique_ptr<TestableConstraint> createTestableBoundsFor(
  std::shared_ptr<Space> _stateSpace)
{
  return detail::createTestableFor_impl<Space>::create(
    std::move(_stateSpace));
}

//=============================================================================
template <class Space>
std::unique_ptr<SampleableConstraint> createSampleableBoundsFor(
  std::shared_ptr<Space> _stateSpace, std::unique_ptr<util::RNG> _rng)
{
  return detail::createSampleableFor_impl<Space>::create(
    std::move(_stateSpace), std::move(_rng));
}

} // namespace constraint
} // namespace aikido
