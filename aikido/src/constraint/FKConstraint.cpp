#include <aikido/constraint/FKConstraint.hpp>

namespace aikido {
namespace constraint {


//=============================================================================
FKConstraint::FKConstraint(
  const dart::dynamics::BodyNodePtr& _bodyNode,
  const DifferentiablePtr& _innerConstraint)
: mBodyNode(_bodyNode)
, mInnerConstraint(_innerConstraint)
{
  if (!mInnerConstraint)
    throw std::invalid_argument("_innerConstraint is nullptr.");

  if (!mBodyNode)
    throw std::invalid_argument("_bodyNode is nullptr.");

}


//=============================================================================
size_t FKConstraint::getConstraintDimension() const
{
  return mInnerConstraint->getConstraintDimension();
}


//=============================================================================
Eigen::VectorXd FKConstraint::getValue(
  const state::StatePtr& _s) const
{
  using state;
  
  RealVectorStatePtr s = std::dynamic_pointer_cast<RealVectorStatePtr>(_s);
  if(!s)
  {
    throw std::invalid_argument("_s is not RealVectorState.");
  }

  mBodyNode->getSkeleton()->setPositions(s->mQ);
  SE3StatePtr eeState = std::make_shared<SE3State>(
                          mBodyNode->getTransform());

  return mInnerConstraint->getValue(eeState);

}


//=============================================================================
state::JacobianPtr FKConstraint::getJacobian(
  const state::StatePtr& _s) const
{
  using state;

  RealVectorStatePtr s = std::dynamic_pointer_cast<RealVectorState>(_s);
  if(!s)
  {
    throw std::invalid_argument("_s is not RealVectorState.");
  }

  mBodyNode->getSkeleton()->setPositions(s->mQ);
  SE3StatePtr eeState = std::make_shared<SE3StatePtr>(
                          mBodyNode->getTransform());
  
  /// SE3JacobianPtr 
  JacobianPtr innerJac = mInnerConstraint->getJacobian(eeState);

  /// Eigen::Matrix<double, 6, stateDimension> 
  dart::math::Jacobian bodyJac = mBodyNode->getJacobian();

  /// Chain rule df/dee*dee/ds
  Eigen::MatrixXd jac((innerJac->mJacobian)*bodyJac);

  return std::make_shared<RealVectorJacobianPtr>(jac);

}


//=============================================================================
std::vector<ConstraintType> FKConstraint::getConstraintTypes() const
{
  return mInnerConstraint->getConstraintTypes();
}
  
}
}
