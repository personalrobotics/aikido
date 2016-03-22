#include <aikido/constraint/ProjectableByDifferentiable.hpp>
#include <limits>

namespace aikido{
namespace constraint{


//=============================================================================
ProjectableByDifferentiable::ProjectableByDifferentiable(
  const DifferentiablePtr& _differentiable) const
: mDifferentiable(_differentiable)
{
}


//=============================================================================
bool contains(const state::State& _s) const
{
  Eigen::VectorXd values = mDifferentiable->getValue(_s);
  std::vector<ConstraintType> types = mDifferentiable->getConstraintTypes();

  for(int i = 0; i < values.size(); i++)
  {
    if (types.at(i) == ConstraintType::EQ)
    {
      // TODO: better way to check value == 0?
      if (std::abs(values.at(i)) > std::numeric_limits<double>::epsilon())
        return false;
    }
    else
    {
      if (values.at(i) > 0)
        return false;
    }
  }

  return true;
}

//=============================================================================
boost::optional<state::State> project(const state::State& _s) const 
{
  if contains(_s)
  {
    return _s;
  }

  // TODO: use newton's method on mDifferentiable
  state::CompoundState s(_s);

  while(!contains(s))
  {
    Eigen::VectorXd value = mDifferentiable->getValue(s);
    std::vector<Eigen::MatrixXd> Jac = mDifferentaible->getJacobian(s);

    for(int i = 0; i < s.components.size(); s++)
    {
      // Real // wouldn't work for non-inversible matrix Jac.
      s.components.at(i).mQ = s.components.at(i).mQ - Jac.at(i).inverse()*value;

      // SO2 

      // SO3
      // get rotation
      s.components.at(i).mRotation = s.components.at(i).mRotation*rotation;


      // SE2

      // SE3

    }

  }

  return s;
}

}
}