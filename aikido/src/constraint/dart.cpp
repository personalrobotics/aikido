#include <aikido/constraint/dart.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
std::unique_ptr<Differentiable> createDifferentiableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace)
{
  return detail::ForOneOf<
      detail::createDifferentiableFor_impl,
      statespace::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

//=============================================================================
std::unique_ptr<Differentiable> createDifferentiableBounds(
  statespace::MetaSkeletonStateSpacePtr _metaSkeleton)
{
  const auto n = _metaSkeleton->getNumStates();

  std::vector<std::shared_ptr<Differentiable>> constraints;
  constraints.reserve(n);

  for (size_t i = 0; i < n; ++i)
  {
    auto subspace = _metaSkeleton->getSubSpace<statespace::JointStateSpace>(i);
    auto constraint = createDifferentiableBounds(std::move(subspace));
    constraints.emplace_back(constraint.release());
  }

  // TODO: Apply a separate constraint to each dimension.

  throw std::runtime_error("not implemented");
}

//=============================================================================
std::unique_ptr<Projectable> createProjectableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace)
{
  return detail::ForOneOf<
      detail::createProjectableFor_impl,
      statespace::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

//=============================================================================
std::unique_ptr<Projectable> createProjectableBounds(
  statespace::MetaSkeletonStateSpacePtr _metaSkeleton)
{
  const auto n = _metaSkeleton->getNumStates();

  std::vector<std::shared_ptr<Projectable>> constraints;
  constraints.reserve(n);

  for (size_t i = 0; i < n; ++i)
  {
    auto subspace = _metaSkeleton->getSubSpace<statespace::JointStateSpace>(i);
    auto constraint = createProjectableBounds(std::move(subspace));
    constraints.emplace_back(constraint.release());
  }

  // TODO: Apply a separate constraint to each dimension.

  throw std::runtime_error("not implemented");
}

//=============================================================================
std::unique_ptr<TestableConstraint> createTestableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace)
{
  return detail::ForOneOf<
      detail::createTestableFor_impl,
      statespace::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

//=============================================================================
std::unique_ptr<TestableConstraint> createTestableBounds(
  statespace::MetaSkeletonStateSpacePtr _metaSkeleton)
{
  const auto n = _metaSkeleton->getNumStates();

  std::vector<std::shared_ptr<TestableConstraint>> constraints;
  constraints.reserve(n);

  for (size_t i = 0; i < n; ++i)
  {
    auto subspace = _metaSkeleton->getSubSpace<statespace::JointStateSpace>(i);
    auto constraint = createTestableBounds(std::move(subspace));
    constraints.emplace_back(constraint.release());
  }

  // TODO: Apply a separate constraint to each dimension.

  throw std::runtime_error("not implemented");
}

//=============================================================================
std::unique_ptr<SampleableConstraint> createSampleableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace,
  std::unique_ptr<util::RNG> _rng)
{
  return detail::ForOneOf<
      detail::createSampleableFor_impl,
      statespace::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace), std::move(_rng));
}

//=============================================================================
std::unique_ptr<SampleableConstraint> createSampleableBounds(
  statespace::MetaSkeletonStateSpacePtr _metaSkeleton,
  std::unique_ptr<util::RNG> _rng)
{
  // TODO: Create N random number generators.
  throw std::runtime_error("not implemented");
}

} // namespace constraint
} // namespace aikido
