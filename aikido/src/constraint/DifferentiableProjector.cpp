#include <aikido/constraint/DifferentiableProjector.hpp>
#include <limits>
#include <aikido/util/PseudoInverse.hpp>
#include <math.h>
#include <dart/math/Geometry.h>
#include <iostream>

namespace aikido{
namespace constraint{

//=============================================================================
DifferentiableProjector::DifferentiableProjector(
  DifferentiablePtr _differentiable,
  int _maxIteration,
  double _tolerance,
  double _minStepSize) 
: mDifferentiable(std::move(_differentiable))
, mMaxIteration(_maxIteration)
, mTolerance(_tolerance)
, mMinStepSize(_minStepSize)
{
}

//=============================================================================
bool DifferentiableProjector::contains(
  const statespace::StateSpace::State* _s) const
{
  Eigen::VectorXd values = mDifferentiable->getValue(_s);
  std::vector<ConstraintType> types = mDifferentiable->getConstraintTypes();

  for(int i = 0; i < values.size(); i++)
  {
    if (types.at(i) == ConstraintType::EQ)
    {
      if (std::abs(values(i)) > mTolerance)
        return false;
    }
    else
    {
      // Inequality constraints are satisfied when value <= 0.
      if (values(i) > mTolerance)
        return false;
    }
  }

  return true;
}

//=============================================================================
bool DifferentiableProjector::project(
  const statespace::StateSpace::State* _s,
  statespace::StateSpace::State* _out) const
{
  using StateSpace = statespace::StateSpace;
  using State = StateSpace::State;

  int iteration = 0;

  statespace::StateSpacePtr space = mDifferentiable->getStateSpace(); 

  space->copyState(_s, _out );

  /// Newton's method on mDifferentiable
  while(!contains(_out) && iteration < mMaxIteration)
  {
    iteration++;

    Eigen::VectorXd value = mDifferentiable->getValue(_out);
    Eigen::MatrixXd jac = mDifferentiable->getJacobian(_out);
    
    // Minimization step in tangent space.
    Eigen::VectorXd tangentStep = -1*util::pseudoinverse(jac)*value;

    // Break if tangent step is too small. 
    if (tangentStep.maxCoeff() < mMinStepSize &&
        (-1*tangentStep).maxCoeff() < mMinStepSize)
      break;

    StateSpace::ScopedState step(space.get());

    // Minimization step in state space.
    space->expMap(tangentStep, step);
    space->compose(_out, step);
  }

  if (!contains(_out))
    return false;

  return true;

}

//=============================================================================
statespace::StateSpacePtr DifferentiableProjector::getStateSpace() const
{
  return mDifferentiable->getStateSpace();
}


} // constraint
} // aikido
