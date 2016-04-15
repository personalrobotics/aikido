#ifndef AIKIDO_CONSTRAINT_FKTESTABLE_H
#define AIKIDO_CONSTRAINT_FKTESTABLE_H

#include "TestableConstraint.hpp"
#include "../statespace/SE3StateSpace.hpp"
#include "../statespace/dart/MetaSkeletonStateSpace.hpp"
#include <dart/dynamics/dynamics.h>

namespace aikido
{
namespace constraint
{
/// Transforms a Isometry3d TestableConstraint into a VectorXd
/// TestableConstraint that checks the positions of a Skeleto by
/// performing forward kinematics on a pose in the robots configuration space
/// and checking the resulting Isometry3d.
class FkTestable : public TestableConstraint
{
public:
  /// Creat a TestableConstraint for the MetaSkeleton
  FkTestable(statespace::MetaSkeletonStateSpacePtr _stateSpace,
             dart::dynamics::Frame* _frame,
             TestableConstraintPtr _poseConstraint);

  /// Check if the constraint is satisfied by performing forward
  ///  kinematics and checking the poseConstraint
  bool isSatisfied(const statespace::StateSpace::State* _state) const override;

  // Documentation inhereted
  std::shared_ptr<statespace::StateSpace> getStateSpace()
      const override;

private:
    statespace::MetaSkeletonStateSpacePtr mStateSpace;
    dart::dynamics::Frame* mFrame;
    TestableConstraintPtr mPoseConstraint;
    std::shared_ptr<statespace::SE3StateSpace> mPoseStateSpace;
};
}
}

#endif
