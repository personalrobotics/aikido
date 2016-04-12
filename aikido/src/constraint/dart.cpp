#include <aikido/constraint/dart.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
std::unique_ptr<Differentiable> GenericJointBoundConstraint
  ::createDifferentiable(StateSpacePtr _stateSpace)
{
  return detail::ForOneOf<
      detail::createDifferentiableFor_impl,
      statespace::JointStateSpace,
      // List of derived types:
      statespace::RealVectorJointStateSpace,
      statespace::SO2JointStateSpace,
      statespace::SO3JointStateSpace,
      statespace::SE2JointStateSpace,
      statespace::SE3JointStateSpace>
    ::create(std::move(_stateSpace));
}

//=============================================================================
std::unique_ptr<Projectable> GenericJointBoundConstraint
  ::createProjectable(StateSpacePtr _stateSpace)
{
  return nullptr;
}

//=============================================================================
std::unique_ptr<SampleableConstraint> GenericJointBoundConstraint
  ::createUniformSampleable(
  StateSpacePtr _stateSpace, std::unique_ptr<util::RNG> _rng)
{
  return nullptr;
}

//=============================================================================
std::unique_ptr<TestableConstraint> GenericJointBoundConstraint
  ::createTestable(StateSpacePtr _stateSpace)
{
  return nullptr;
}

} // namespace constraint
} // namespace aikido
