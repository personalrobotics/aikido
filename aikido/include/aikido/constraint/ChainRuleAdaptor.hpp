#ifndef AIKIDO_CONSTRAINT_CHAINRULEADAPTOR_H
#define AIKIDO_CONSTRAINT_CHAINRULEADAPTOR_H

#include <dart/dynamics/dynamics.h>
#include "Differentiable.hpp"
#include <Eigen/Dense>

namespace aikido {
namespace constraint{

/// Constrains multiple bodynodes' transforms.
/// Each body node is controlled by its correponding metaskeleton.
class ChainRuleAdaptor: public Differentiable
{
public:

  using BodySkeletonPair = std::pair<const dart::dynamics::BodyNodePtr,
                                     const dart::dynamics::MetaSkeletonPtr>;

  /// _constraint should take CompoundStatePtr.
  /// States comprising CompoundState should be SE3States,
  /// corresponding to the transform of bodyNode in _bodySkeletonPairs.
  ChainRuleAdaptor(const DifferentiablePtr& _constraint,
    std::vector<BodySkeletonPair> _bodySkeletonPairs);                 

  /// Size of constraints
  size_t getConstraintDimension() const override;

  /// _s should be CompoundState, and each state should be 
  /// RealVectorState for generalized coordinates of 
  /// metaskeletons in mBodySkeletonPairs. 
  /// Size of _s.mComponents must be equal to size of mBodySkeletonPairs.
  Eigen::VectorXd getValue(const state::StatePtr& _s) const override;


  /// _s should be CompoundState with each component mapping to 
  /// each BodySkeletonPair. Returns CompoundJacobianPtr.
  state::JacobianPtr getJacobian(
    const state::StatePtr& _s) const override;


  /// Returns a vector containing each constraint's type.
  std::vector<ConstraintType> getConstraintTypes() const override;

private:
  
  /// Set all Skeleton dofs by _s' values and
  /// return a CompoundState containing all SE3States of bodyNodes.
  state::StatePtr getInnerState(const state::StatePtr& _s) const;

  std::vector<BodySkeletonPair> mBodySkeletonPairs;
  DifferentiablePtr mConstraint;


};


} // constraint
} // aikido

#endif