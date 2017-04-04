#include <sstream>
#include <dart/common/StlHelpers.hpp>
#include "../../statespace/dart/RnJoint.hpp"
#include "../../statespace/dart/SO2Joint.hpp"
#include "../../statespace/dart/SO3Joint.hpp"
#include "../../statespace/dart/SE2Joint.hpp"
#include "../../statespace/dart/SE3Joint.hpp"
#include "../../statespace/dart/WeldJoint.hpp"
#include "../../util/metaprogramming.hpp"
#include "../uniform/RnBoxConstraint.hpp"
#include "../uniform/SO2UniformSampler.hpp"
#include "../uniform/SO3UniformSampler.hpp"
#include "../uniform/RnConstantSampler.hpp"
#include "../Satisfied.hpp"

namespace aikido {
namespace constraint {
namespace detail {

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

//=============================================================================
inline Eigen::VectorXd getPositionLowerLimits(
  const dart::dynamics::Joint* _joint)
{
  const auto dimension = _joint->getNumDofs();
  Eigen::VectorXd bounds(dimension);

  for (size_t i = 0; i < dimension; ++i)
    bounds[i] = _joint->getPositionLowerLimit(i);

  return bounds;
}

//=============================================================================
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
  statespace::dart::RnJoint,
  statespace::dart::SO2Joint,
  statespace::dart::SO3Joint,
  statespace::dart::SE2Joint,
  statespace::dart::SE3Joint,
  statespace::dart::WeldJoint
>;

//=============================================================================
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
  std::shared_ptr<statespace::dart::RnJoint> _stateSpace,
  std::unique_ptr<util::RNG> _rng)
{
  const auto joint = _stateSpace->getJoint();

  if (isLimited(joint))
    return dart::common::make_unique<RnBoxConstraint>(
      std::move(_stateSpace), std::move(_rng),
      getPositionLowerLimits(joint), getPositionUpperLimits(joint));
  else
    return dart::common::make_unique<Satisfied>(
      std::move(_stateSpace));
}

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::RnJoint>
{
  using StateSpace = statespace::dart::RnJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Differentiable>(std::move(_stateSpace), nullptr);
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::dart::RnJoint>
{
  using StateSpace = statespace::dart::RnJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Testable>(
      std::move(_stateSpace), nullptr);
  }
};

//=============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::RnJoint>
{
  using StateSpace = statespace::dart::RnJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Projectable>(
      std::move(_stateSpace), nullptr);
  }
};

//=============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::RnJoint>
{
  using StateSpace = statespace::dart::RnJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    const auto joint = _stateSpace->getJoint();

    if (isLimited(joint))
      return dart::common::make_unique<RnBoxConstraint>(
        std::move(_stateSpace), std::move(_rng),
        getPositionLowerLimits(joint), getPositionUpperLimits(joint));
    else
      throw std::runtime_error(
        "Unable to create Sampleable for unbounded Rn.");
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2Joint must not have limits.");

    return dart::common::make_unique<
      SO2Sampleable>(
        std::move(_stateSpace), std::move(_rng));
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(
      std::move(_stateSpace));
  }
};

//=============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3Joint must not have limits.");

    return dart::common::make_unique<SO3UniformSampler>(
      std::move(_stateSpace), std::move(_rng));
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No DifferentiableConstraint is available for SE2Joint.");
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No Testable is available for SE2Joint.");
  }
};

//=============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No Projectable is available for SE2Joint.");
  }
};

//=============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    throw std::runtime_error(
      "No Sampleable is available for SE2Joint.");
  }
};

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No DifferentiableConstraint is available for SE3Joint.");
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No Testable is available for SE3Joint.");
  }
};

//=============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    throw std::runtime_error(
      "No Projectable is available for SE3Joint.");
  }
};

//=============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
  {
    throw std::runtime_error(
      "No Sampleable is available for SE3Joint.");
  }
};

//=============================================================================
template <class OutputConstraint>
std::unique_ptr<OutputConstraint> createBoxConstraint(
  std::shared_ptr<statespace::dart::WeldJoint> _stateSpace,
  std::unique_ptr<util::RNG> /*_rng*/)
{
  return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
}

//=============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Differentiable>(std::move(_stateSpace), nullptr);
  }
};

//=============================================================================
template <>
struct createTestableFor_impl<statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Testable>(std::move(_stateSpace), nullptr);
  }
};

//=============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<Projectable>(std::move(_stateSpace), nullptr);
  }
};

//=============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
    StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> /*_rng*/)
  {
    const auto joint = _stateSpace->getJoint();
    Eigen::VectorXd positions = joint->getPositions();

    return dart::common::make_unique<RnConstantSampler>(
        std::move(_stateSpace), positions);
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
std::unique_ptr<Testable> createTestableBoundsFor(
  std::shared_ptr<Space> _stateSpace)
{
  return detail::createTestableFor_impl<Space>::create(
    std::move(_stateSpace));
}

//=============================================================================
template <class Space>
std::unique_ptr<Sampleable> createSampleableBoundsFor(
  std::shared_ptr<Space> _stateSpace, std::unique_ptr<util::RNG> _rng)
{
  return detail::createSampleableFor_impl<Space>::create(
    std::move(_stateSpace), std::move(_rng));
}

} // namespace constraint
} // namespace aikido
