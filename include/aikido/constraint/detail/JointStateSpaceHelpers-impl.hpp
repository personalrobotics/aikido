#include <sstream>
#include <dart/common/StlHelpers.hpp>
#include "../../common/metaprogramming.hpp"
#include "../../statespace/dart/RnJoint.hpp"
#include "../../statespace/dart/SE2Joint.hpp"
#include "../../statespace/dart/SE3Joint.hpp"
#include "../../statespace/dart/SO2Joint.hpp"
#include "../../statespace/dart/SO3Joint.hpp"
#include "../../statespace/dart/WeldJoint.hpp"
#include "../Satisfied.hpp"
#include "../uniform/RnBoxConstraint.hpp"
#include "../uniform/RnConstantSampler.hpp"
#include "../uniform/SE2BoxConstraint.hpp"
#include "../uniform/SO2UniformSampler.hpp"
#include "../uniform/SO3UniformSampler.hpp"

namespace aikido {
namespace constraint {
namespace detail {

//==============================================================================
inline bool isLimited(const dart::dynamics::Joint* _joint)
{
  for (size_t i = 0; i < _joint->getNumDofs(); ++i)
  {
    if (_joint->hasPositionLimit(i))
      return true;
  }
  return false;
}

//==============================================================================
inline Eigen::VectorXd getPositionLowerLimits(
    const dart::dynamics::Joint* _joint)
{
  const auto dimension = _joint->getNumDofs();
  Eigen::VectorXd bounds(dimension);

  for (size_t i = 0; i < dimension; ++i)
    bounds[i] = _joint->getPositionLowerLimit(i);

  return bounds;
}

//==============================================================================
inline Eigen::VectorXd getPositionUpperLimits(
    const dart::dynamics::Joint* _joint)
{
  const auto dimension = _joint->getNumDofs();
  Eigen::VectorXd bounds(dimension);

  for (size_t i = 0; i < dimension; ++i)
    bounds[i] = _joint->getPositionUpperLimit(i);

  return bounds;
}

//==============================================================================
using JointStateSpaceTypeList = common::type_list<statespace::dart::R0Joint,
                                                  statespace::dart::R1Joint,
                                                  statespace::dart::R2Joint,
                                                  statespace::dart::R3Joint,
                                                  statespace::dart::R6Joint,
                                                  statespace::dart::SO2Joint,
                                                  statespace::dart::SO3Joint,
                                                  statespace::dart::SE2Joint,
                                                  statespace::dart::SE3Joint,
                                                  statespace::dart::WeldJoint>;

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
    std::shared_ptr<statespace::dart::RJoint<N>> _stateSpace,
    std::unique_ptr<common::RNG> _rng)
{
  const auto joint = _stateSpace->getJoint();

  if (isLimited(joint))
  {
    return dart::common::make_unique<RBoxConstraint<N>>(
        std::move(_stateSpace),
        std::move(_rng),
        getPositionLowerLimits(joint),
        getPositionUpperLimits(joint));
  }
  else
  {
    return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
}

//==============================================================================
template <int N>
struct createDifferentiableFor_impl<statespace::dart::RJoint<N>>
{
  using StateSpace = statespace::dart::RJoint<N>;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<N, Differentiable>(
        std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <int N>
struct createTestableFor_impl<statespace::dart::RJoint<N>>
{
  using StateSpace = statespace::dart::RJoint<N>;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<N, Testable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <int N>
struct createProjectableFor_impl<statespace::dart::RJoint<N>>
{
  using StateSpace = statespace::dart::RJoint<N>;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    return createBoxConstraint<N, Projectable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <int N>
struct createSampleableFor_impl<statespace::dart::RJoint<N>>
{
  using StateSpace = statespace::dart::RJoint<N>;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
      StateSpacePtr _stateSpace, std::unique_ptr<common::RNG> _rng)
  {
    const auto joint = _stateSpace->getJoint();

    if (isLimited(joint))
    {
      return dart::common::make_unique<RBoxConstraint<N>>(
          std::move(_stateSpace),
          std::move(_rng),
          getPositionLowerLimits(joint),
          getPositionUpperLimits(joint));
    }
    else
    {
      throw std::runtime_error("Unable to create Sampleable for unbounded Rn.");
    }
  }
};

//==============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::SO2Joint>
{
  using StateSpace = statespace::dart::SO2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
      StateSpacePtr _stateSpace, std::unique_ptr<common::RNG> _rng)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO2Joint must not have limits.");

    return dart::common::make_unique<SO2UniformSampler>(
        std::move(_stateSpace), std::move(_rng));
  }
};

//==============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3Joint must not have limits.");

    return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::SO3Joint>
{
  using StateSpace = statespace::dart::SO3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
      StateSpacePtr _stateSpace, std::unique_ptr<common::RNG> _rng)
  {
    if (isLimited(_stateSpace->getJoint()))
      throw std::invalid_argument("SO3Joint must not have limits.");

    return dart::common::make_unique<SO3UniformSampler>(
        std::move(_stateSpace), std::move(_rng));
  }
};

//==============================================================================
template <class OutputConstraint>
std::unique_ptr<OutputConstraint> createBoxConstraint(
    std::shared_ptr<statespace::dart::SE2Joint> _stateSpace,
    std::unique_ptr<common::RNG> _rng)
{
  const auto joint = _stateSpace->getJoint();

  if (isLimited(joint))
  {
    return dart::common::make_unique<SE2BoxConstraint>(
        std::move(_stateSpace),
        std::move(_rng),
        getPositionLowerLimits(joint).tail<2>(),
        getPositionUpperLimits(joint).tail<2>());
  }
  else
  {
    return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
  }
}

//==============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr /*_stateSpace*/)
  {
    throw std::runtime_error(
        "No DifferentiableConstraint is available for SE2Joint.");
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr _stateSpace)
  {
    auto joint = _stateSpace->getJoint();
    if (joint->hasPositionLimit(0))
    {
      throw std::invalid_argument(
          "Rotational component of SE2Joint must not have limits.");
    }

    return createBoxConstraint<Testable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr _stateSpace)
  {
    auto joint = _stateSpace->getJoint();
    if (joint->hasPositionLimit(0))
    {
      throw std::invalid_argument(
          "Rotational component of SE2Joint must not have limits.");
    }

    return createBoxConstraint<Projectable>(std::move(_stateSpace), nullptr);
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::SE2Joint>
{
  using StateSpace = statespace::dart::SE2Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
      StateSpacePtr stateSpace, std::unique_ptr<common::RNG> rng)
  {
    auto joint = stateSpace->getJoint();
    if (joint->hasPositionLimit(0))
    {
      throw std::invalid_argument(
          "Rotational component of SE2Joint must not have limits.");
    }
    else if (!(joint->hasPositionLimit(1) && joint->hasPositionLimit(2)))
    {
      throw std::runtime_error(
          "Unable to create Sampleable for unbounded SE2.");
    }
    else
    {
      return dart::common::make_unique<SE2BoxConstraint>(
          std::move(stateSpace),
          std::move(rng),
          getPositionLowerLimits(joint).tail<2>(),
          getPositionUpperLimits(joint).tail<2>());
    }
  }
};

//==============================================================================
template <>
struct createDifferentiableFor_impl<statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Differentiable> create(StateSpacePtr /*_stateSpace*/)
  {
    throw std::runtime_error(
        "No DifferentiableConstraint is available for SE3Joint.");
  }
};

//==============================================================================
template <>
struct createTestableFor_impl<statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Testable> create(StateSpacePtr /*_stateSpace*/)
  {
    throw std::runtime_error("No Testable is available for SE3Joint.");
  }
};

//==============================================================================
template <>
struct createProjectableFor_impl<statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Projectable> create(StateSpacePtr /*_stateSpace*/)
  {
    throw std::runtime_error("No Projectable is available for SE3Joint.");
  }
};

//==============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::SE3Joint>
{
  using StateSpace = statespace::dart::SE3Joint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
      StateSpacePtr /*_stateSpace*/, std::unique_ptr<common::RNG> /*_rng*/)
  {
    throw std::runtime_error("No Sampleable is available for SE3Joint.");
  }
};

//==============================================================================
template <class OutputConstraint>
std::unique_ptr<OutputConstraint> createBoxConstraint(
    std::shared_ptr<statespace::dart::WeldJoint> _stateSpace,
    std::unique_ptr<common::RNG> /*_rng*/)
{
  return dart::common::make_unique<Satisfied>(std::move(_stateSpace));
}

//==============================================================================
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

//==============================================================================
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

//==============================================================================
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

//==============================================================================
template <>
struct createSampleableFor_impl<statespace::dart::WeldJoint>
{
  using StateSpace = statespace::dart::WeldJoint;
  using StateSpacePtr = std::shared_ptr<StateSpace>;

  static std::unique_ptr<Sampleable> create(
      StateSpacePtr _stateSpace, std::unique_ptr<common::RNG> /*_rng*/)
  {
    const auto joint = _stateSpace->getJoint();
    Eigen::VectorXd positions = joint->getPositions();

    return dart::common::make_unique<R0ConstantSampler>(
        std::move(_stateSpace), positions);
  }
};

} // namespace detail

//==============================================================================
template <class Space>
std::unique_ptr<Differentiable> createDifferentiableBoundsFor(
    std::shared_ptr<Space> _stateSpace)
{
  return detail::createDifferentiableFor_impl<Space>::create(
      std::move(_stateSpace));
}

//==============================================================================
template <class Space>
std::unique_ptr<Projectable> createProjectableBoundsFor(
    std::shared_ptr<Space> _stateSpace)
{
  return detail::createProjectableFor_impl<Space>::create(
      std::move(_stateSpace));
}

//==============================================================================
template <class Space>
std::unique_ptr<Testable> createTestableBoundsFor(
    std::shared_ptr<Space> _stateSpace)
{
  return detail::createTestableFor_impl<Space>::create(std::move(_stateSpace));
}

//==============================================================================
template <class Space>
std::unique_ptr<Sampleable> createSampleableBoundsFor(
    std::shared_ptr<Space> _stateSpace, std::unique_ptr<common::RNG> _rng)
{
  return detail::createSampleableFor_impl<Space>::create(
      std::move(_stateSpace), std::move(_rng));
}

} // namespace constraint
} // namespace aikido
