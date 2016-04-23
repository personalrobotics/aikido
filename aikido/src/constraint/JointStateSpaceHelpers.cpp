#include <aikido/constraint/JointStateSpaceHelpers.hpp>
#include <aikido/constraint/DifferentiableSubSpace.hpp>
#include <aikido/constraint/DifferentiableIntersection.hpp>
#include <aikido/constraint/TestableIntersection.hpp>
#include <aikido/constraint/CartesianProductProjectable.hpp>
#include <aikido/constraint/CartesianProductSampleable.hpp>
#include <aikido/constraint/TestableSubspace.hpp>
#include <dart/common/StlHelpers.h>

namespace aikido {
namespace constraint {

using dart::common::make_unique;

//=============================================================================
std::unique_ptr<Differentiable> createDifferentiableBounds(
  std::shared_ptr<statespace::dart::JointStateSpace> _stateSpace)
{
  return util::DynamicCastFactory<
      detail::createDifferentiableFor_impl,
      util::DynamicCastFactory_shared_ptr,
      statespace::dart::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

//=============================================================================
std::unique_ptr<Differentiable> createDifferentiableBounds(
  statespace::dart::MetaSkeletonStateSpacePtr _metaSkeleton)
{
  if (!_metaSkeleton)
    throw std::invalid_argument("MetaSkeletonStateSpace is nullptr.");

  const auto n = _metaSkeleton->getNumStates();

  std::vector<std::shared_ptr<Differentiable>> constraints;
  constraints.reserve(n);

  // TODO: Filter out trivial constraints for efficiency.
  for (size_t i = 0; i < n; ++i)
  {
    auto subspace = _metaSkeleton->getSubSpace<statespace::dart::JointStateSpace>(i);
    auto subSpaceConstraint = createDifferentiableBounds(std::move(subspace));
    auto constraint = std::make_shared<DifferentiableSubSpace>(
      _metaSkeleton, std::move(subSpaceConstraint), i);
    constraints.emplace_back(std::move(constraint));
  }

  // TODO: We should std::move constraints here, but we can't because
  // DifferentiableIntersection does not take by value.
  return make_unique<DifferentiableIntersection>(constraints, _metaSkeleton);
}

//=============================================================================
std::unique_ptr<Projectable> createProjectableBounds(
  std::shared_ptr<statespace::dart::JointStateSpace> _stateSpace)
{
  return util::DynamicCastFactory<
      detail::createProjectableFor_impl,
      util::DynamicCastFactory_shared_ptr,
      statespace::dart::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

//=============================================================================
std::unique_ptr<Projectable> createProjectableBounds(
  statespace::dart::MetaSkeletonStateSpacePtr _metaSkeleton)
{
  const auto n = _metaSkeleton->getNumStates();

  std::vector<ProjectablePtr> constraints;
  constraints.reserve(n);

  for (size_t i = 0; i < n; ++i)
  {
    auto subspace = _metaSkeleton->getSubSpace<statespace::dart::JointStateSpace>(i);
    auto constraint = createProjectableBounds(std::move(subspace));
    constraints.emplace_back(constraint.release());
  }

  return make_unique<CartesianProductProjectable>(
    std::move(_metaSkeleton), std::move(constraints));
}

//=============================================================================
std::unique_ptr<Testable> createTestableBounds(
  std::shared_ptr<statespace::dart::JointStateSpace> _stateSpace)
{
  return util::DynamicCastFactory<
      detail::createTestableFor_impl,
      util::DynamicCastFactory_shared_ptr,
      statespace::dart::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace));
}

//=============================================================================
std::unique_ptr<Testable> createTestableBounds(
  statespace::dart::MetaSkeletonStateSpacePtr _metaSkeleton)
{
  const auto n = _metaSkeleton->getNumStates();

  std::vector<std::shared_ptr<Testable>> constraints;
  constraints.reserve(n);

  for (size_t i = 0; i < n; ++i)
  {
    auto subspace = _metaSkeleton->getSubSpace<statespace::dart::JointStateSpace>(i);
    auto constraint = createTestableBounds(std::move(subspace));
    constraints.emplace_back(constraint.release());
  }

  return make_unique<TestableSubspace>(
    std::move(_metaSkeleton), std::move(constraints));
}

//=============================================================================
std::unique_ptr<Sampleable> createSampleableBounds(
  std::shared_ptr<statespace::dart::JointStateSpace> _stateSpace,
  std::unique_ptr<util::RNG> _rng)
{
  return util::DynamicCastFactory<
      detail::createSampleableFor_impl,
      util::DynamicCastFactory_shared_ptr,
      statespace::dart::JointStateSpace,
      detail::JointStateSpaceTypeList
    >::create(std::move(_stateSpace), std::move(_rng));
}

//=============================================================================
std::unique_ptr<Sampleable> createSampleableBounds(
  statespace::dart::MetaSkeletonStateSpacePtr _metaSkeleton,
  std::unique_ptr<util::RNG> _rng)
{
  const auto n = _metaSkeleton->getNumStates();

  // Create a new RNG for each subspace.
  auto engines = splitEngine(*_rng, n, util::NUM_DEFAULT_SEEDS);

  std::vector<std::shared_ptr<Sampleable>> constraints;
  constraints.reserve(n);

  for (size_t i = 0; i < n; ++i)
  {
    auto subspace = _metaSkeleton->getSubSpace<statespace::dart::JointStateSpace>(i);
    auto constraint = createSampleableBounds(
      std::move(subspace), std::move(engines[i]));
    constraints.emplace_back(constraint.release());
  }

  return make_unique<CartesianProductSampleable>(
    std::move(_metaSkeleton), std::move(constraints));
}

} // namespace constraint
} // namespace aikido
