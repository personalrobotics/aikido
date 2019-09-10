#include <sstream>

#include "aikido/common/memory.hpp"
#include "aikido/common/metaprogramming.hpp"
#include "aikido/constraint/Satisfied.hpp"
#include "aikido/constraint/uniform/RnBoxConstraint.hpp"
#include "aikido/constraint/uniform/RnConstantSampler.hpp"
#include "aikido/constraint/uniform/SE2BoxConstraint.hpp"
#include "aikido/constraint/uniform/SO2UniformSampler.hpp"
#include "aikido/constraint/uniform/SO3UniformSampler.hpp"
#include "aikido/statespace/dart/RnJoint.hpp"
#include "aikido/statespace/dart/SE2Joint.hpp"
#include "aikido/statespace/dart/SE3Joint.hpp"
#include "aikido/statespace/dart/SO2Joint.hpp"
#include "aikido/statespace/dart/SO3Joint.hpp"
#include "aikido/statespace/dart/WeldJoint.hpp"

namespace aikido {
namespace constraint {
namespace dart {
namespace detail {

//==============================================================================
using JointStateSpaceTypeList = common::type_list<
    const statespace::dart::R0Joint,
    const statespace::dart::R1Joint,
    const statespace::dart::R2Joint,
    const statespace::dart::R3Joint,
    const statespace::dart::R6Joint,
    const statespace::dart::SO2Joint,
    const statespace::dart::SO3Joint,
    const statespace::dart::SE2Joint,
    const statespace::dart::SE3Joint,
    const statespace::dart::WeldJoint>;

//==============================================================================
template <class T>
struct createDifferentiableFor_impl
{
  // Nothing defined.
};

template <class T>
struct createTestableFor_impl
{
  // Nothing defined.
};

template <class T>
struct createProjectableFor_impl
{
  // Nothing defined.
};

template <class T>
struct createSampleableFor_impl
{
  // Nothing defined.
};

//==============================================================================
template <int N, class OutputConstraint>
std::unique_ptr<OutputConstraint> createBoxConstraint(
    std::shared_ptr<const statespace::dart::RJoint<N>> _stateSpace,
    std::unique_ptr<common::RNG> _rng)
{
  const auto properties = _stateSpace->getProperties();

  if (properties.isPositionLimited())
  {
    return ::aikido::common::make_unique<uniform::RBoxConstraint<N>>(
        std::move(_stateSpace),
        std::move(_rng),
        properties.getPositionLowerLimits(),
        properties.getPositionUpperLimits());
  }
  else
  {
    return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
}

//==============================================================================
template <int N>
struct createDifferentiableFor_impl<const statespace::dart::RJoint<N>>
{
  using StateSpace = statespace::dart::RJoint<N>;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Differentiable> create(ConstStateSpacePtr _stateSpace)
  {
    return createBoxConstraint<N, Differentiable>(
        std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <int N>
struct createTestableFor_impl<const statespace::dart::RJoint<N>>
{
  using StateSpace = statespace::dart::RJoint<N>;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Testable> create(ConstStateSpacePtr _stateSpace)
  {
    return createBoxConstraint<N, Testable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <int N>
struct createProjectableFor_impl<const statespace::dart::RJoint<N>>
{
  using StateSpace = statespace::dart::RJoint<N>;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Projectable> create(ConstStateSpacePtr _stateSpace)
  {
    return createBoxConstraint<N, Projectable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <int N>
struct createSampleableFor_impl<const statespace::dart::RJoint<N>>
{
  using StateSpace = statespace::dart::RJoint<N>;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Sampleable> create(
      ConstStateSpacePtr _stateSpace, std::unique_ptr<common::RNG> _rng)
  {
    const auto properties = _stateSpace->getProperties();

    if (properties.isPositionLimited())
    {
      return ::aikido::common::make_unique<uniform::RBoxConstraint<N>>(
          std::move(_stateSpace),
          std::move(_rng),
          properties.getPositionLowerLimits(),
          properties.getPositionUpperLimits());
    }
    else
    {
      throw std::runtime_error("Unable to create Sampleable for unbounded Rn.");
    }
  }
};

//==============================================================================
template <>
struct createDifferentiableFor_impl<const statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Differentiable> create(ConstStateSpacePtr _stateSpace)
  {
    if (_stateSpace->getProperties().isPositionLimited())
      throw std::invalid_argument("SO2Joint must not have limits.");

    return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<const statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Testable> create(ConstStateSpacePtr _stateSpace)
  {
    if (_stateSpace->getProperties().isPositionLimited())
      throw std::invalid_argument("SO2Joint must not have limits.");

    return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<const statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Projectable> create(ConstStateSpacePtr _stateSpace)
  {
    if (_stateSpace->getProperties().isPositionLimited())
      throw std::invalid_argument("SO2Joint must not have limits.");

    return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<const statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Sampleable> create(
      ConstStateSpacePtr _stateSpace, std::unique_ptr<common::RNG> _rng)
  {
    if (_stateSpace->getProperties().isPositionLimited())
      throw std::invalid_argument("SO2Joint must not have limits.");

    return ::aikido::common::make_unique<uniform::SO2UniformSampler>(
        std::move(_stateSpace), std::move(_rng));
  }
};

//==============================================================================
template <>
struct createDifferentiableFor_impl<const statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Differentiable> create(ConstStateSpacePtr _stateSpace)
  {
    if (_stateSpace->getProperties().isPositionLimited())
      throw std::invalid_argument("SO3Joint must not have limits.");

    return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<const statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Testable> create(ConstStateSpacePtr _stateSpace)
  {
    if (_stateSpace->getProperties().isPositionLimited())
      throw std::invalid_argument("SO3Joint must not have limits.");

    return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<const statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Projectable> create(ConstStateSpacePtr _stateSpace)
  {
    if (_stateSpace->getProperties().isPositionLimited())
      throw std::invalid_argument("SO3Joint must not have limits.");

    return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<const statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Sampleable> create(
      ConstStateSpacePtr _stateSpace, std::unique_ptr<common::RNG> _rng)
  {
    if (_stateSpace->getProperties().isPositionLimited())
      throw std::invalid_argument("SO3Joint must not have limits.");

    return ::aikido::common::make_unique<uniform::SO3UniformSampler>(
        std::move(_stateSpace), std::move(_rng));
  }
};

//==============================================================================
template <class OutputConstraint>
std::unique_ptr<OutputConstraint> createBoxConstraint(
    std::shared_ptr<const statespace::dart::SE2Joint> _stateSpace,
    std::unique_ptr<common::RNG> _rng)
{
  const auto properties = _stateSpace->getProperties();

  if (properties.isPositionLimited())
  {
    return ::aikido::common::make_unique<uniform::SE2BoxConstraint>(
        std::move(_stateSpace),
        std::move(_rng),
        properties.getPositionLowerLimits().tail<2>(),
        properties.getPositionUpperLimits().tail<2>());
  }
  else
  {
    return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
}

//==============================================================================
template <>
struct createDifferentiableFor_impl<const statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Differentiable> create(
      ConstStateSpacePtr /*_stateSpace*/)
  {
    throw std::runtime_error(
        "No DifferentiableConstraint is available for SE2Joint.");
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<const statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Testable> create(ConstStateSpacePtr _stateSpace)
  {
    const auto properties = _stateSpace->getProperties();
    if (properties.hasPositionLimit(0))
    {
      throw std::invalid_argument(
          "Rotational component of SE2Joint must not have limits.");
    }

    return createBoxConstraint<Testable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<const statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Projectable> create(ConstStateSpacePtr _stateSpace)
  {
    const auto properties = _stateSpace->getProperties();
    if (properties.hasPositionLimit(0))
    {
      throw std::invalid_argument(
          "Rotational component of SE2Joint must not have limits.");
    }

    return createBoxConstraint<Projectable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<const statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Sampleable> create(
      ConstStateSpacePtr stateSpace, std::unique_ptr<common::RNG> rng)
  {
    const auto properties = stateSpace->getProperties();
    if (properties.hasPositionLimit(0))
    {
      throw std::invalid_argument(
          "Rotational component of SE2Joint must not have limits.");
    }
    else if (!(properties.hasPositionLimit(1)
               && properties.hasPositionLimit(2)))
    {
      throw std::runtime_error(
          "Unable to create Sampleable for unbounded SE2.");
    }
    else
    {
      return ::aikido::common::make_unique<uniform::SE2BoxConstraint>(
          std::move(stateSpace),
          std::move(rng),
          properties.getPositionLowerLimits().tail<2>(),
          properties.getPositionUpperLimits().tail<2>());
    }
  }
};

//==============================================================================
template <>
struct createDifferentiableFor_impl<const statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Differentiable> create(
      ConstStateSpacePtr /*_stateSpace*/)
  {
    throw std::runtime_error(
        "No DifferentiableConstraint is available for SE3Joint.");
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<const statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Testable> create(ConstStateSpacePtr /*_stateSpace*/)
  {
    throw std::runtime_error("No Testable is available for SE3Joint.");
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<const statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Projectable> create(ConstStateSpacePtr /*_stateSpace*/)
  {
    throw std::runtime_error("No Projectable is available for SE3Joint.");
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<const statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Sampleable> create(
      ConstStateSpacePtr /*_stateSpace*/, std::unique_ptr<common::RNG> /*_rng*/)
  {
    throw std::runtime_error("No Sampleable is available for SE3Joint.");
  }
};

//==============================================================================
template <class OutputConstraint>
std::unique_ptr<OutputConstraint> createBoxConstraint(
    std::shared_ptr<const statespace::dart::WeldJoint> _stateSpace,
    std::unique_ptr<common::RNG> /*_rng*/)
{
  return ::aikido::common::make_unique<Satisfied>(std::move(_stateSpace));
}

//==============================================================================
template <>
struct createDifferentiableFor_impl<const statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Differentiable> create(ConstStateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Differentiable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<const statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Testable> create(ConstStateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Testable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<const statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Projectable> create(ConstStateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Projectable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<const statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using ConstStateSpacePtr = std::shared_ptr<const StateSpace>;

  static std::unique_ptr<Sampleable> create(
      ConstStateSpacePtr _stateSpace, std::unique_ptr<common::RNG> /*_rng*/)
  {
    // A WeldJoint has zero DOFs
    Eigen::VectorXd positions = Eigen::Matrix<double, 0, 1>();

    return ::aikido::common::make_unique<uniform::R0ConstantSampler>(
        std::move(_stateSpace), positions);
  }
};

} // namespace detail

//==============================================================================
template <class Space>
std::unique_ptr<Differentiable> createDifferentiableBoundsFor(
    std::shared_ptr<Space> _stateSpace)
{
  return detail::createDifferentiableFor_impl<const Space>::create(
      std::move(_stateSpace));
}

//==============================================================================
template <class Space>
std::unique_ptr<Projectable> createProjectableBoundsFor(
    std::shared_ptr<Space> _stateSpace)
{
  return detail::createProjectableFor_impl<const Space>::create(
      std::move(_stateSpace));
}

//==============================================================================
template <class Space>
std::unique_ptr<Testable> createTestableBoundsFor(
    std::shared_ptr<Space> _stateSpace)
{
  return detail::createTestableFor_impl<const Space>::create(
      std::move(_stateSpace));
}

//==============================================================================
template <class Space>
std::unique_ptr<Sampleable> createSampleableBoundsFor(
    std::shared_ptr<Space> _stateSpace, std::unique_ptr<common::RNG> _rng)
{
  return detail::createSampleableFor_impl<const Space>::create(
      std::move(_stateSpace), std::move(_rng));
}

} // namespace dart
} // namespace constraint
} // namespace aikido
