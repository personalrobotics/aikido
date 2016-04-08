#include <aikido/constraint/DifferentiableProjector.hpp>
#include <limits>
#include <aikido/util/PseudoInverse.hpp>
#include <math.h>

#include <dart/math/Geometry.h>

namespace aikido{
namespace constraint{

//=============================================================================
DifferentiableProjector::DifferentiableProjector(
  const DifferentiablePtr& _differentiable,
  int _maxIteration) 
: mDifferentiable(_differentiable)
, mMaxIteration(_maxIteration)
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
      // TODO: better way to check value == 0?
      if (std::abs(values(i)) > std::numeric_limits<double>::epsilon())
        return false;
    }
    else
    {
      if (values(i) > 0)
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

  space->copyState(_out, _s);

  /// Newton's method on mDifferentiable
  while(!contains(_out) && iteration < mMaxIteration)
  {
    iteration++;

    Eigen::VectorXd value = mDifferentiable->getValue(_out);
    Eigen::MatrixXd jac = mDifferentiable->getJacobian(_out);
    
    Eigen::VectorXd tangent = -1*util::pseudoinverse(jac);

    StateSpace::ScopedState update(space.get());

    space->expMap(tangent*value, update);
    space->compose(_out, update, _out);
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
