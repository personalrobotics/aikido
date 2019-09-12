#include "aikido/constraint/dart/JointStateSpaceHelpers.hpp"

#include "aikido/common/memory.hpp"
#include "aikido/constraint/CartesianProductProjectable.hpp"
#include "aikido/constraint/CartesianProductSampleable.hpp"
#include "aikido/constraint/CartesianProductTestable.hpp"
#include "aikido/constraint/DifferentiableIntersection.hpp"
#include "aikido/constraint/DifferentiableSubspace.hpp"
#include "aikido/constraint/TestableIntersection.hpp"

namespace aikido {
namespace constraint {
namespace dart {

//==============================================================================
std::unique_ptr<Differentiable> createDifferentiableBounds(
    std::shared_ptr<const statespace::dart::JointStateSpace> _stateSpace)
{
  return common::DynamicCastFactory<
      detail::createDifferentiableFor_impl,
      common::DynamicCastFactory_shared_ptr,
      const statespace::dart::JointStateSpace,
      detail::JointStateSpaceTypeList>::create(std::move(_stateSpace));
}

//==============================================================================
std::unique_ptr<Differentiable> createDifferentiableBounds(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeleton)
{
  if (!_metaSkeleton)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  const auto n = _metaSkeleton->getNumSubspaces();

  std::vector<std::shared_ptr<Differentiable>> constraints;
  constraints.reserve(n);

  // TODO: Filter out trivial constraints for efficiency.
  for (std::size_t i = 0; i < n; ++i)
  {
    auto subspace
        = _metaSkeleton->getSubspace<statespace::dart::JointStateSpace>(i);
    auto subSpaceConstraint = createDifferentiableBounds(std::move(subspace));
    auto constraint = std::make_shared<DifferentiableSubspace>(
        _metaSkeleton, std::move(subSpaceConstraint), i);
    constraints.emplace_back(std::move(constraint));
  }

  // TODO: We should std::move constraints here, but we can't because
  // DifferentiableIntersection does not take by value.
  return ::aikido::common::make_unique<DifferentiableIntersection>(
      constraints, _metaSkeleton);
}

//==============================================================================
std::unique_ptr<Projectable> createProjectableBounds(
    std::shared_ptr<const statespace::dart::JointStateSpace> _stateSpace)
{
  return common::DynamicCastFactory<
      detail::createProjectableFor_impl,
      common::DynamicCastFactory_shared_ptr,
      const statespace::dart::JointStateSpace,
      detail::JointStateSpaceTypeList>::create(std::move(_stateSpace));
}

//==============================================================================
std::unique_ptr<Projectable> createProjectableBounds(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeleton)
{
  const auto n = _metaSkeleton->getNumSubspaces();

  std::vector<ProjectablePtr> constraints;
  constraints.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    auto subspace
        = _metaSkeleton->getSubspace<statespace::dart::JointStateSpace>(i);
    auto constraint = createProjectableBounds(std::move(subspace));
    constraints.emplace_back(constraint.release());
  }

  return ::aikido::common::make_unique<CartesianProductProjectable>(
      std::move(_metaSkeleton), std::move(constraints));
}

//==============================================================================
std::unique_ptr<Testable> createTestableBounds(
    std::shared_ptr<const statespace::dart::JointStateSpace> _stateSpace)
{
  return common::DynamicCastFactory<
      detail::createTestableFor_impl,
      common::DynamicCastFactory_shared_ptr,
      const statespace::dart::JointStateSpace,
      detail::JointStateSpaceTypeList>::create(std::move(_stateSpace));
}

//==============================================================================
std::unique_ptr<Testable> createTestableBounds(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeleton)
{
  const auto n = _metaSkeleton->getNumSubspaces();

  std::vector<std::shared_ptr<const Testable>> constraints;
  constraints.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    auto subspace
        = _metaSkeleton->getSubspace<statespace::dart::JointStateSpace>(i);
    auto constraint = createTestableBounds(std::move(subspace));
    constraints.emplace_back(constraint.release());
  }

  return ::aikido::common::make_unique<CartesianProductTestable>(
      std::move(_metaSkeleton), std::move(constraints));
}

//==============================================================================
std::unique_ptr<Sampleable> createSampleableBounds(
    std::shared_ptr<const statespace::dart::JointStateSpace> _stateSpace,
    std::unique_ptr<common::RNG> _rng)
{
  return common::DynamicCastFactory<
      detail::createSampleableFor_impl,
      common::DynamicCastFactory_shared_ptr,
      const statespace::dart::JointStateSpace,
      detail::JointStateSpaceTypeList>::
      create(std::move(_stateSpace), std::move(_rng));
}

//==============================================================================
std::unique_ptr<Sampleable> createSampleableBounds(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeleton,
    std::unique_ptr<common::RNG> _rng)
{
  const auto n = _metaSkeleton->getNumSubspaces();

  // Create a new RNG for each subspace.
  auto engines = cloneRNGsFrom(*_rng, n, common::NUM_DEFAULT_SEEDS);

  std::vector<std::shared_ptr<Sampleable>> constraints;
  constraints.reserve(n);

  for (std::size_t i = 0; i < n; ++i)
  {
    auto subspace
        = _metaSkeleton->getSubspace<statespace::dart::JointStateSpace>(i);
    auto constraint
        = createSampleableBounds(std::move(subspace), std::move(engines[i]));
    constraints.emplace_back(constraint.release());
  }

  return ::aikido::common::make_unique<CartesianProductSampleable>(
      std::move(_metaSkeleton), std::move(constraints));
}

} // namespace dart
} // namespace constraint
} // namespace aikido
