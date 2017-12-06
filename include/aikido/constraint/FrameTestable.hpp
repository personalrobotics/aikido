#ifndef AIKIDO_CONSTRAINT_FRAMETESTABLE_HPP_
#define AIKIDO_CONSTRAINT_FRAMETESTABLE_HPP_

#include <dart/dynamics/dynamics.hpp>
#include "../statespace/SE3.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include "Testable.hpp"

namespace aikido {
namespace constraint {

/// Transforms a SE(3) Testable into a MetaSkeleton-Testable by
/// performing forward kinematics on a configuration (metaskeleton state)
/// and checking the resulting SE(3) pose of the asked frame.
class FrameTestable : public Testable
{
public:
  /// Creat a Testable for the MetaSkeleton.
  /// \param _stateSpace Configuration space of the metaskeleton.
  /// \param _frame Frame constrained by _poseConstraint.
  /// \param _poseConstraint A testable constraint on _frame.
  FrameTestable(
      statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
      dart::dynamics::ConstJacobianNodePtr _frame,
      TestablePtr _poseConstraint);

  /// Check if the constraint is satisfied by performing forward
  /// kinematics and testing the poseConstraint.
  /// \param _state a MetaskeletonState to set the configuration of this
  ///        constraint's metaskeketon. This state's StateSpace should match
  ///        StateSpace returend by getStateSpace().
  bool isSatisfied(
      const statespace::StateSpace::State* _state,
      TestableOutcome* outcome = nullptr) const override;

  /// Return an instance of DefaultOutcome, since this class doesn't have a
  /// more specialized TestableOutcome derivative assigned to it.
  std::unique_ptr<TestableOutcome> createOutcome() const override;

  // Documentation inhereted
  std::shared_ptr<statespace::StateSpace> getStateSpace() const override;

private:
  statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
  dart::dynamics::ConstJacobianNodePtr mFrame;
  TestablePtr mPoseConstraint;
  std::shared_ptr<statespace::SE3> mPoseStateSpace;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_FRAMETESTABLE_HPP_
