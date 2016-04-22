#include "PolynomialConstraint.hpp"
#include <memory>

//=============================================================================
PolynomialConstraint::PolynomialConstraint(Eigen::VectorXd _coeffs)
: mCoeffs(_coeffs)
, mStateSpace(std::make_shared<aikido::statespace::RealVectorStateSpace>(1))
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
    const aikido::statespace::StateSpace::State* _s) const
{
  using State = aikido::statespace::RealVectorStateSpace::State;
  auto s = static_cast<const State*>(_s);

  double x = mStateSpace->getValue(s)(0);
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
    const aikido::statespace::StateSpace::State* _s) const
{
  using State = aikido::statespace::RealVectorStateSpace::State;
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
std::pair<Eigen::VectorXd, Eigen::MatrixXd> PolynomialConstraint::getValueAndJacobian(
    const aikido::statespace::StateSpace::State* _s) const
{
  using State = aikido::statespace::RealVectorStateSpace::State;
  auto s = static_cast<const State*>(_s);

  Eigen::VectorXd value = getValue(s);
  Eigen::MatrixXd jacobian = getJacobian(s); 

  return std::make_pair(value, jacobian);

}


//=============================================================================
std::vector<aikido::constraint::ConstraintType>
PolynomialConstraint::getConstraintTypes() const
{
  std::vector<aikido::constraint::ConstraintType> ctypes;
  ctypes.push_back(aikido::constraint::ConstraintType::EQUALITY);
  return ctypes;
}

//=============================================================================
aikido::statespace::StateSpacePtr PolynomialConstraint::getStateSpace() const
{
  return mStateSpace;
}

