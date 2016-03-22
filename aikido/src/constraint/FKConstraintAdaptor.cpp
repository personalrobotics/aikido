#include <aikido/constraint/FKConstraintAdaptor.hpp>

namespace aikido {
namespace constraint {

//=============================================================================
FKConstraintAdaptor::FKConstraintAdaptor(
  const dart::dynamics::BodyNodePtr& _bodyNode,
  const DifferentiablePtr& _innerConstraint)
: mBodyNode(_bodyNode)
, mInnerConstraint(_innerConstraint)
{
}

//=============================================================================
size_t FKConstraintAdaptor::getConstraintDimension() const
{
  return mInnerConstraint->getConstraintDimension();
}


  /// _s should be RealVectorState for generalized coordinates' positions
Eigen::VectorXd FKConstraintAdaptor::getValue(
  const state::CompoundState& _s) const
{
  assert(_s.components().size() == 1);
  state::RealVectorState* rvState = static_cast<RealVectorState*> s.components().at(0);
  assert(rvState!=nullptr);

  mBodyNode->getSkeleton()->setPositions(rvState->mQ);
  state::SE3State eeState(mBodyNode->getTransform());
  

}


  /// Jacobian of constraints at _s.
  /// For SO3 state, returns mx3 matrix.
  /// For SO2 state, returns mx1 matrix.
  /// For vector<n> state, returns mxn matrix.
  /// For compound state with k components, returns k matrices 
  /// where ith value correcsponds to Jacobian at ith component.
  vector<Eigen::MatrixXd> getJacobian(
    const state::CompoundState& _s) const override;


  /// Returns a vector containing each constraint's type.
  std::vector<ConstraintType> getConstraintTypes() const override;
  
}
}