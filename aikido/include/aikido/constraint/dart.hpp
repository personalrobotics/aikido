#ifndef AIKIDO_CONSTRAINT_DART_HPP_
#define AIKIDO_CONSTRAINT_DART_HPP_
#include "Differentiable.hpp"
#include "Projectable.hpp"
#include "Sampleable.hpp"
#include "TestableConstraint.hpp"
#include "../statespace/dart/JointStateSpace.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"


namespace aikido {
namespace constraint {

template <class Space>
std::unique_ptr<Differentiable> createDifferentiableBoundsFor(
  std::shared_ptr<Space> _stateSpace);

std::unique_ptr<Differentiable> createDifferentiableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace);

std::unique_ptr<Differentiable> createDifferentiableBounds(
  statespace::MetaSkeletonStateSpacePtr _metaSkeleton);

template <class Space>
std::unique_ptr<Projectable> createProjectableBoundsFor(
  std::shared_ptr<Space> _stateSpace);

std::unique_ptr<Projectable> createProjectableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace);

std::unique_ptr<Projectable> createProjectableBounds(
  statespace::MetaSkeletonStateSpacePtr _metaSkeleton);

template <class Space>
std::unique_ptr<TestableConstraint> createTestableBoundsFor(
  std::shared_ptr<Space> _stateSpace);

std::unique_ptr<TestableConstraint> createTestableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace);

std::unique_ptr<TestableConstraint> createTestableBounds(
  statespace::MetaSkeletonStateSpacePtr _metaSkeleton);

template <class Space>
std::unique_ptr<SampleableConstraint> createSampleableBoundsFor(
  std::shared_ptr<Space> _stateSpace, std::unique_ptr<util::RNG> _rng);

std::unique_ptr<SampleableConstraint> createSampleableBounds(
  std::shared_ptr<statespace::JointStateSpace> _stateSpace,
  std::unique_ptr<util::RNG> _rng);

std::unique_ptr<SampleableConstraint> createSampleableBounds(
  statespace::MetaSkeletonStateSpacePtr _metaSkeleton,
  std::unique_ptr<util::RNG> _rng);

} // namespace constraint
} // namespace aikido

#include "detail/dart.hpp"

#endif // ifndef AIKIDO_CONSTRAINT_DART_HPP_
