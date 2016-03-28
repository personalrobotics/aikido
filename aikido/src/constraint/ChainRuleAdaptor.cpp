#include <aikido/constraint/ChainRuleAdaptor.hpp>

namespace aikido{
namespace constraint{

//=============================================================================
ChainRuleAdaptor::ChainRuleAdaptor(
  const DifferentiablePtr& _constraint,
  std::vector<BodySkeletonPair> _bodySkeletonPairs)
: mConstraint(_constraint)
, mBodySkeletonPairs(_bodySkeletonPairs)
{

}


//=============================================================================
size_t ChainRuleAdaptor::getConstraintDimension() const
{
  return mConstraint->getConstraintDimension();
}


//=============================================================================
Eigen::VectorXd ChainRuleAdaptor::getValue(const state::StatePtr& _s) const
{
  state::StatePtr innerState = getInnerState(_s);
  return mConstraint->getValue(innerState);
}

//=============================================================================
state::JacobianPtr ChainRuleAdaptor::getJacobian(const state::StatePtr& _s) const
{
  using namespace state;
  StatePtr innerState = getInnerState(_s);
  JacobianPtr _innerJs = mConstraint->getJacobian(innerState);

  CompoundJacobianPtr innerJs = std::dynamic_pointer_cast
                                <CompoundJacobian>(_innerJs);
  if (!innerJs)
    throw std::invalid_argument("mConstraint's Jacobian is not CompoundJacobian.");
  if (innerJs->mJacobian.size() != mBodySkeletonPairs.size())
    throw std::invalid_argument("mConstraint's Jacobian should have equal"
                                "number of components as mBodySkeletonPairs.");


  std::vector<JacobianPtr> chains;
  chains.reserve(mBodySkeletonPairs.size());

  // Chain jacobians.

  for (int i = 0; i < mBodySkeletonPairs.size(); ++i)
  {
    dart::dynamics::BodyNodePtr bodyNode = mBodySkeletonPairs.at(i).first;

    // dSE3/dq; Eigen::Matrix<double, 6, numDofs> 
    dart::math::Jacobian bodyJac = bodyNode->getJacobian(); 

    // Chain rule df/dSE3*dSE3/dq
    SE3JacobianPtr innerJ = std::dynamic_pointer_cast<SE3Jacobian> (innerJs->mJacobian.at(i));
    Eigen::MatrixXd chainJac(innerJ->mJacobian*bodyJac);
    chains.emplace_back(std::make_shared<RealVectorJacobian>(chainJac));
  }

  return std::make_shared<CompoundJacobian>(chains);
}

//=============================================================================
state::StatePtr ChainRuleAdaptor::getInnerState(const state::StatePtr& _s) const
{
  using namespace state;

  CompoundStatePtr s = std::dynamic_pointer_cast<CompoundState>(_s);
  if (!s)
    throw std::invalid_argument("_s shoud be CompoundStatePtr.");

  if (s->mComponents.size() != mBodySkeletonPairs.size())
    throw std::invalid_argument("_s should have equal number of"
                                "components as mBodySkeletonPairs.");


  std::vector<StatePtr> innerStates;
  innerStates.reserve(s->mComponents.size());

  // Set each skeleton's joint positions according to _s.
  for (int i = 0; i < mBodySkeletonPairs.size(); ++i)
  {
    RealVectorStatePtr rvState = std::dynamic_pointer_cast<RealVectorState>(
                                   s->mComponents.at(i));

    if (!rvState)
      throw std::invalid_argument("_s's components should all be RealVectorState.");

    dart::dynamics::MetaSkeletonPtr skeleton = mBodySkeletonPairs.at(i).second;

    if (rvState->mQ.size() != skeleton->getNumDofs())
      throw std::invalid_argument("_s's dimension does not match with skeleton.");

    skeleton->setPositions(rvState->mQ);

    SE3StatePtr se3State = std::make_shared<SE3State>(
                             mBodySkeletonPairs.at(i).first->getTransform());
    innerStates.emplace_back(se3State);
  }

  return std::make_shared<CompoundState>(innerStates);

}

//=============================================================================
std::vector<ConstraintType> ChainRuleAdaptor::getConstraintTypes() const
{
  return mConstraint->getConstraintTypes();
}

}
}
