#include <aikido/constraint/ProjectableByDifferentiable.hpp>
#include <limits>
#include <aikido/util/PseudoInverse.hpp>
#include <aikido/state/Jacobian.hpp>
#include <math.h>

#include <dart/math/Geometry.h>

namespace aikido{
namespace constraint{


//=============================================================================
ProjectableByDifferentiable::ProjectableByDifferentiable(
  const std::shared_ptr<const Differentiable>& _differentiable,
  int _maxIteration) 
: mDifferentiable(_differentiable)
, mMaxIteration(_maxIteration)
{
}

//=============================================================================
bool ProjectableByDifferentiable::contains(const state::StatePtr& _s) const
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
boost::optional<state::StatePtr> ProjectableByDifferentiable::project(
  const state::StatePtr& _s)  
{
  using namespace state;

  int iteration = 0;

  // Copy of _s's state
  StatePtr sCopy = _s->clone();

  if(!sCopy)
  {
    throw std::invalid_argument("_s is not clonable.");
  } 


  /// Newton's method on mDifferentiable
  while(!contains(sCopy) && iteration < mMaxIteration)
  {
    iteration++;
    Eigen::VectorXd value = mDifferentiable->getValue(sCopy);
    JacobianPtr jac = mDifferentiable->getJacobian(sCopy);
    
    // Real
    RealVectorStatePtr sRV = std::dynamic_pointer_cast<RealVectorState>(sCopy);
    RealVectorJacobianPtr jacRV = std::dynamic_pointer_cast<
                                  RealVectorJacobian>(jac);
    if (sRV && jacRV)
    {
      Eigen::VectorXd ds = -1*util::pseudoinverse(jacRV->mJacobian)*value;
      sRV->update(std::make_shared<RealVectorState>(ds));
      continue;
    }

    // SO2 
    SO2StatePtr sSO2 = std::dynamic_pointer_cast<SO2State>(sCopy);
    SO2JacobianPtr jacSO2 = std::dynamic_pointer_cast<SO2Jacobian>(jac);
    if (sSO2 && jacSO2)
    {
      Eigen::MatrixXd ds = -1*util::pseudoinverse(jacSO2->mJacobian)*value;
      Eigen::Rotation2D<double> jacRot(ds(0,0));
      sSO2->update(std::make_shared<SO2State>(jacRot));
      continue;
    }

    // SE2 :: TODO: dart::expMap takes only Vec6d
    // SE2StatePtr sSE2 = std::dynamic_pointer_cast<SE2State>(sCopy);
    // SE2JacobianPtr jacSE2 = std::dynamic_pointer_cast<SE2Jacobian>(jac);
    // if (sSE2 && jacSE2)
    // {
    //   Eigen::VectorXd _ds = -1*util::pseudoinverse(jacSE2->mJacobian)*value;
    //   Eigen::Isometry2d ds = ::dart::math::expMap(_ds);
    //   sSE2->update(std::make_shared<SE2State>(ds));
    //   continue;
    // }

    // SO3 : TODO 

    // SE3 
    SE3StatePtr sSE3 = std::dynamic_pointer_cast<SE3State>(sCopy);
    SE3JacobianPtr jacSE3 = std::dynamic_pointer_cast<SE3Jacobian>(jac);
    if (sSE3 && jacSE3)
    {
      Eigen::VectorXd _ds = -1*util::pseudoinverse(jacSE3->mJacobian)*value;
      Eigen::Isometry3d ds = ::dart::math::expMap(_ds);
      sSE3->update(std::make_shared<SE3State>(ds));
      continue;
    }


  }


  return sCopy;
}

}
}