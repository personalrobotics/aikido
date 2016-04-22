#ifndef AIKIDO_CONSTRAINT_FKTESTABLE_HPP_
#define AIKIDO_CONSTRAINT_FKTESTABLE_HPP_

#include "Testable.hpp"
#include "../statespace/SE3.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include <dart/dynamics/dynamics.h>

namespace aikido {
namespace constraint {

/// Transforms a SE(3) Testable into a MetaSkeleton-Testable by
/// performing forward kinematics on a configuration (metaskeleton state)
/// and checking the resulting SE(3) pose of the asked frame.
class FkTestable : public Testable
{
public:
  /// Creat a Testable for the MetaSkeleton.
  /// \param _stateSpace Configuration space of the metaskeleton.
  /// \param _frame Frame constrained by _poseConstraint.
  /// \param _poseConstraint A testable constraint on _frame.
  FkTestable(statespace::dart::MetaSkeletonStateSpacePtr _stateSpace,
             dart::dynamics::ConstJacobianNodePtr _frame,
             TestablePtr _poseConstraint);

  /// Check if the constraint is satisfied by performing forward
  /// kinematics and testing the poseConstraint.
  /// \param _state a MetaskeletonState to set the configuration of this
  ///        constraint's metaskeketon. This state's StateSpace should match 
  ///        StateSpace returend by getStateSpace().
  bool isSatisfied(const statespace::StateSpace::State* _state) const override;

  // Documentation inhereted
  std::shared_ptr<statespace::StateSpace> getStateSpace()
      const override;

private:
    statespace::dart::MetaSkeletonStateSpacePtr mStateSpace;
    dart::dynamics::ConstJacobianNodePtr mFrame;
    TestablePtr mPoseConstraint;
    std::shared_ptr<statespace::SE3> mPoseStateSpace;
};

}
}

#endif
