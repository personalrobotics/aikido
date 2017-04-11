#include <aikido/constraint/NewtonsMethodProjectable.hpp>
#include <limits>
#include <aikido/util/PseudoInverse.hpp>
#include <math.h>
#include <dart/math/Geometry.hpp>
#include <iostream>

namespace aikido{
namespace constraint{

//=============================================================================
NewtonsMethodProjectable::NewtonsMethodProjectable(
  DifferentiablePtr _differentiable, std::vector<double> _tolerance,
  int _maxIteration, double _minStepSize)
: mDifferentiable(std::move(_differentiable))
, mTolerance(std::move(_tolerance))
, mMaxIteration(_maxIteration)
, mMinStepSize(_minStepSize)
{
  if (!mDifferentiable)
    throw std::invalid_argument("_differentiable is nullptr.");

  if (mDifferentiable->getConstraintDimension() != mTolerance.size())
  {
    std::stringstream msg;
    msg << "Number of tolerances does not match the number of constraints:"
        << " expected " << mDifferentiable->getConstraintDimension() << ", got "
        << mTolerance.size();
    throw std::invalid_argument(msg.str());
  }

  for (double tolerance : mTolerance)
  {
    if (tolerance <= 0)
      throw std::invalid_argument("Tolerance should be positive.");
  }

  if (mMaxIteration <= 0)
    throw std::invalid_argument("_maxIteration should be positive.");

  if (mMinStepSize <= 0)
    throw std::invalid_argument("_minStepsize should be positive.");

  mStateSpace = mDifferentiable->getStateSpace();
}

//=============================================================================
bool NewtonsMethodProjectable::contains(
  const statespace::StateSpace::State* _s) const
{
  Eigen::VectorXd values;
  mDifferentiable->getValue(_s, values);
  std::vector<ConstraintType> types = mDifferentiable->getConstraintTypes();

  for(int i = 0; i < values.size(); i++)
  {
    if (types.at(i) == ConstraintType::EQUALITY)
    {
      if (std::abs(values(i)) > mTolerance.at(i))
        return false;
    }
    else
    {
      // Inequality constraints are satisfied when value <= 0.
      if (values(i) > mTolerance.at(i))
        return false;
    }
  }

  return true;
}

//=============================================================================
bool NewtonsMethodProjectable::project(
  const statespace::StateSpace::State* _s,
  statespace::StateSpace::State* _out) const
{
  using StateSpace = statespace::StateSpace;

  int iteration = 0;

  // Initialize _out.
  mStateSpace->copyState(_s, _out);

  StateSpace::ScopedState step(mStateSpace.get());

  /// Newton's method on mDifferentiable
  while(!contains(_out) && iteration < mMaxIteration)
  {
    iteration++;

    Eigen::VectorXd value;
    mDifferentiable->getValue(_out, value);
    Eigen::MatrixXd jac;
    mDifferentiable->getJacobian(_out, jac);
    
    // Minimization step in tangent space.
    Eigen::VectorXd tangentStep = -1*util::pseudoinverse(jac)*value;

    // Break if tangent step is too small. 
    if (tangentStep.maxCoeff() < mMinStepSize &&
        (-1*tangentStep).maxCoeff() < mMinStepSize)
      break;


    // Minimization step in state space.
    mStateSpace->expMap(tangentStep, step);
    mStateSpace->compose(_out, step);
  }

  if (!contains(_out))
    return false;

  return true;

}

//=============================================================================
statespace::StateSpacePtr NewtonsMethodProjectable::getStateSpace() const
{
  return mDifferentiable->getStateSpace();
}


} // constraint
} // aikido
