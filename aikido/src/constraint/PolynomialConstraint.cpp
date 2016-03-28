#include <aikido/constraint/PolynomialConstraint.hpp>
#include <limits>

namespace aikido {
namespace constraint{

//=============================================================================
PolynomialConstraint::PolynomialConstraint(Eigen::VectorXd _coeffs)
: mCoeffs(_coeffs)
{
  if(abs(mCoeffs(mCoeffs.rows()-1)) < std::numeric_limits<double>::epsilon())
  {
    throw std::invalid_argument("_coeffs last element is zero.");
  }
}


//=============================================================================
size_t PolynomialConstraint::getConstraintDimension() const
{
  return 1;
}

//=============================================================================
Eigen::VectorXd PolynomialConstraint::getValue(const state::StatePtr& _s) const
{
  using namespace state;
  RealVectorStatePtr s = std::dynamic_pointer_cast<RealVectorState>(_s);
  if (!s)
  {
    throw std::invalid_argument("_s is not RealVectorState.");
  }else if (s->mQ.rows() != 1)
  {
    throw std::invalid_argument("_s's dimension must be 1.");
  }
  else
  {
    double val = 0; 
    double x = s->mQ(0);
    /*if (abs(x) > std::numeric_limits<double>::epsilon())
    {*/
      for(int i = 0; i < mCoeffs.rows(); i++)
      {
        val += mCoeffs(i)*pow(x, i);
      }
    //}

    Eigen::VectorXd value(1);
    value(0) = val;

    return value;
  }

}


//=============================================================================
state::JacobianPtr PolynomialConstraint::getJacobian(
  const state::StatePtr& _s) const
{
  using namespace state;
  RealVectorStatePtr s = std::dynamic_pointer_cast<RealVectorState>(_s);
  if (!s)
    throw std::invalid_argument("_s is not RealVectorState.");

  Eigen::VectorXd derivCoeffs(mCoeffs.rows()-1);
  for(int i = 0; i < derivCoeffs.rows(); i++)
  {
    derivCoeffs(i) = mCoeffs(i+1)*(i+1);
  }

  PolynomialConstraint derivPoly(derivCoeffs);


  Eigen::VectorXd jac = derivPoly.getValue(_s);
  return std::make_shared<RealVectorJacobian>(jac);

} 


//=============================================================================
std::vector<ConstraintType> PolynomialConstraint::getConstraintTypes() const
{ 
  std::vector<ConstraintType> ctypes;
  ctypes.push_back(ConstraintType::EQ);
  return ctypes;
}

}
}
