#include <aikido/constraint/PolynomialConstraint.hpp>

#include <memory>

namespace aikido {
namespace constraint{

//=============================================================================
PolynomialConstraint::PolynomialConstraint(Eigen::VectorXd _coeffs)
: mCoeffs(_coeffs)
, mStateSpace(statespace::RealVectorStateSpace(1))
{
  if(std::abs(mCoeffs(mCoeffs.rows()-1)) < std::numeric_limits<double>::epsilon())
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
Eigen::VectorXd PolynomialConstraint::getValue(
  const statespace::StateSpace::State* _s) const
{
  using State = statespace::RealVectorStateSpace::State;
  auto s = static_cast<const State*>(_s);

  double x = mStateSpace.getValue(s)(0);
  double val = 0; 
  
  for(int i = 0; i < mCoeffs.rows(); i++)
  {
    val += mCoeffs(i)*pow(x, i);
  }

  Eigen::VectorXd value(1);
  value(0) = val;

  return value;
}


//=============================================================================
Eigen::MatrixXd PolynomialConstraint::getJacobian(
  const statespace::StateSpace::State* _s) const
{
  using State = statespace::RealVectorStateSpace::State;
  auto s = static_cast<const State*>(_s);

  Eigen::VectorXd derivCoeffs(mCoeffs.rows()-1);
  for(int i = 0; i < derivCoeffs.rows(); i++)
  {
    derivCoeffs(i) = mCoeffs(i+1)*(i+1);
  }

  PolynomialConstraint derivPoly(derivCoeffs);

  return derivPoly.getValue(_s);
} 


//=============================================================================
std::vector<ConstraintType> PolynomialConstraint::getConstraintTypes() const
{ 
  std::vector<ConstraintType> ctypes;
  ctypes.push_back(ConstraintType::EQ);
  return ctypes;
}

//=============================================================================
statespace::StateSpacePtr PolynomialConstraint::getStateSpace() const
{
  return std::make_shared<statespace::RealVectorStateSpace>(mStateSpace);
}

}
}
