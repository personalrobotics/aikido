#ifndef AIKIDO_CONSTRAINT_DART_JOINTSTATESPACEHELPERS_HPP_
#define AIKIDO_CONSTRAINT_DART_JOINTSTATESPACEHELPERS_HPP_

#include "aikido/constraint/Differentiable.hpp"
#include "aikido/constraint/Projectable.hpp"
#include "aikido/constraint/Sampleable.hpp"
#include "aikido/constraint/Testable.hpp"
#include "aikido/statespace/dart/JointStateSpace.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace constraint {
namespace dart {

/// Create differentiable bounds that can be applied to the given StateSpace
/// \param _stateSpace The StateSpace where the Differentiable will be applied
template <class Space>
std::unique_ptr<Differentiable> createDifferentiableBoundsFor(
    std::shared_ptr<Space> _stateSpace);

/// Create differentiable bounds that can be applied to the given StateSpace
/// The differentiable bounds are created from the joint limits set on the dart
/// joint wrapped by the JointStateSpace.  If the space has no bounds, a no-op
/// Differtiable is created.
/// \param _stateSpace The StateSpace where the Differentiable will be applied
std::unique_ptr<Differentiable> createDifferentiableBounds(
    std::shared_ptr<const statespace::dart::JointStateSpace> _stateSpace);

/// Create a set of Differentiable constraints for each joint in the
/// MetaSkeleton wrapped by the state space.  The bounds are created from the
/// joint limits set on the joints of the MetaSkeleton. A no-op Differentiable
/// is created for joints that have no limits.
/// \param _metaSkeleton The MetaSkeletonStateSpace where the Differentiable
/// will be applied
std::unique_ptr<Differentiable> createDifferentiableBounds(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeleton);

/// Create a Projectable that can be used to project a state from the given
/// StateSpace back within bounds set on the StateSpace.
/// \param _stateSpace The StateSpace where the Projectable will be applied.
template <class Space>
std::unique_ptr<Projectable> createProjectableBoundsFor(
    std::shared_ptr<Space> _stateSpace);

/// Create a Projectable that can be used to project the value of a joint back
/// within joint limits. The Projectable is created from the joint limits set
/// for the joint wrapped by the given JointStateSpace.
/// \param _stateSpace The JointStateSpace where the Projectable will be
/// applied.
std::unique_ptr<Projectable> createProjectableBounds(
    std::shared_ptr<const statespace::dart::JointStateSpace> _stateSpace);

/// Create a set of Projectable constraints for each joint in the MetaSkeleton
/// wrapped by the state space. The constraints can be used to project a state
/// back within the joint limits set on the MetaSkeleton.
/// \param _metaSkeleton The MetaSkeletonStateSpace where the Projectable will
/// be applied.
std::unique_ptr<Projectable> createProjectableBounds(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeleton);

/// Create a Testable constraint that can be used to determine if a given state
/// lies within bounds set on the StateSpace.
/// \param _stateSpace The StateSpace where the Testable will be applied
template <class Space>
std::unique_ptr<Testable> createTestableBoundsFor(
    std::shared_ptr<Space> _stateSpace);

/// Create a Testable constraint that can be used to determine if the value of a
/// joint is within the joint limits. The Testable is created from the joint
/// limits set on the joint wrapped by the given JointStateSpace.
/// \param _stateSpace The JointStateSpace where the Testable will be applied
std::unique_ptr<Testable> createTestableBounds(
    std::shared_ptr<const statespace::dart::JointStateSpace> _stateSpace);

/// Create a set of Testable constraints for each joint in the MetaSkeleton
/// wrapped by the state space.  The constraints can be used to determine if a
/// state is within the joint limits defined on all joints of the MetaSkeleton.
/// \param _metaSkeleton The MetaSkeletonStateSpace where the Testable will be
/// applied.
std::unique_ptr<Testable> createTestableBounds(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeleton);

/// Create a Sampleable constraint that can be used to sample a state that is
/// guarenteed to lie within bounds define on the StateSpace.
/// \param _stateSpace The StateSpace where the Sampleable will be applied.
/// \param _rng The random number generator to be used by the Sampleable
template <class Space>
std::unique_ptr<Sampleable> createSampleableBoundsFor(
    std::shared_ptr<Space> _stateSpace, std::unique_ptr<common::RNG> _rng);

/// Create a Sampleabe constraint that can be used to sample values for the
/// joint that are guarenteed to be within joint limits.  The Sampleable is
/// created from the joint limits set on the joint wrapped by the given
/// JointStateSpace.
/// \param _stateSpace The JointStateSpace where the Sampleable will be applied
/// \param _rng The random number generator to be used by the Sampleable
std::unique_ptr<Sampleable> createSampleableBounds(
    std::shared_ptr<const statespace::dart::JointStateSpace> _stateSpace,
    std::unique_ptr<common::RNG> _rng);

/// Create a Sampleable constraint that can be used to sampel values for all
/// joints in a MetaSkeleton. The sampled joint values are guarenteed to lie
/// within joint limits defined on the MetaSkeleton.
/// \param _metaSkeleton The MetaSkeletonStateSpace where the Sampleable will be
/// applied
/// \param _rng The random number generator to be used by the Sampleable
std::unique_ptr<Sampleable> createSampleableBounds(
    statespace::dart::ConstMetaSkeletonStateSpacePtr _metaSkeleton,
    std::unique_ptr<common::RNG> _rng);

} // namespace dart
} // namespace constraint
} // namespace aikido

#include "detail/JointStateSpaceHelpers-impl.hpp"

#endif // AIKIDO_CONSTRAINT_DART_JOINTSTATESPACEHELPERS_HPP_
