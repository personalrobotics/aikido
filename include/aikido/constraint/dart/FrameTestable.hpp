#ifndef AIKIDO_CONSTRAINT_DART_FRAMETESTABLE_HPP_
#define AIKIDO_CONSTRAINT_DART_FRAMETESTABLE_HPP_

#include <dart/dynamics/dynamics.hpp>

#include "aikido/constraint/Testable.hpp"
#include "aikido/statespace/SE3.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace constraint {
namespace dart {

/// Transforms a SE(3) Testable into a MetaSkeleton-Testable by
/// performing forward kinematics on a configuration (metaskeleton state)
/// and checking the resulting SE(3) pose of the asked frame.
class FrameTestable : public Testable
{
public:
  /// Create a Testable for the MetaSkeleton.
  /// \param _metaSkeletonStateSpace Configuration space of the metaskeleton.
  /// \param _metaskeleton MetaSkeleton to test with
  /// \param _frame Frame constrained by _poseConstraint.
  /// \param _poseConstraint A testable constraint on _frame.
  FrameTestable(
      statespace::dart::MetaSkeletonStateSpacePtr _metaSkeletonStateSpace,
      ::dart::dynamics::MetaSkeletonPtr _metaskeleton,
      ::dart::dynamics::ConstJacobianNodePtr _frame,
      TestablePtr _poseConstraint);

  /// Check if the constraint is satisfied by performing forward
  /// kinematics and testing the poseConstraint.
  /// \param _state a MetaskeletonState to set the configuration of this
  ///        constraint's metaskeketon. This state's StateSpace should match
  ///        StateSpace returend by getStateSpace().
  /// \param outcome Testable outcome derivative class. Passed to the
  ///        isSatisfied method of mPoseConstraint to allow "pass through" of
  ///        debugging information.
  bool isSatisfied(
      const statespace::StateSpace::State* _state,
      TestableOutcome* outcome = nullptr) const override;

  /// Return the outcome of mPoseConstraint->createOutcome(). Reason:
  /// isSatisfied in this class will just pass outcome to the isSatisfied
  /// method of of mPoseConstraint.
  std::unique_ptr<TestableOutcome> createOutcome() const override;

  // Documentation inherited
  statespace::ConstStateSpacePtr getStateSpace() const override;

private:
  statespace::dart::ConstMetaSkeletonStateSpacePtr mMetaSkeletonStateSpace;
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  ::dart::dynamics::ConstJacobianNodePtr mFrame;
  TestablePtr mPoseConstraint;
  std::shared_ptr<const statespace::SE3> mPoseStateSpace;
};

} // namespace dart
} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_DART_FRAMETESTABLE_HPP_
