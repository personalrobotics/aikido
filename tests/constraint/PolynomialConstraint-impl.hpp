#include <memory>

#include "PolynomialConstraint.hpp"

//==============================================================================
template <int N>
PolynomialConstraint<N>::PolynomialConstraint(
    const Eigen::VectorXd& _coeffs,
    std::shared_ptr<aikido::statespace::R<N>> _space)
  : mCoeffs(_coeffs), mStateSpace(std::move(_space))
{
  if (std::abs(mCoeffs(mCoeffs.rows() - 1))
      < std::numeric_limits<double>::epsilon())
  {
    throw std::invalid_argument("_coeffs last element is zero.");
  }

  if (!mStateSpace)
    throw std::invalid_argument("StateSpace is null.");
}

//==============================================================================
template <int N>
std::size_t PolynomialConstraint<N>::getConstraintDimension() const
{
  return 1;
}

//==============================================================================
template <int N>
void PolynomialConstraint<N>::getValue(
    const aikido::statespace::StateSpace::State* _s,
    Eigen::VectorXd& _out) const
{
  using State = typename aikido::statespace::R<N>::State;
  auto s = static_cast<const State*>(_s);

  double x = mStateSpace->getValue(s)(0);
  double val = 0;

  for (int i = 0; i < mCoeffs.rows(); i++)
    val += mCoeffs(i) * pow(x, i);

  _out.resize(1);
  _out(0) = val;
}

//==============================================================================
template <int N>
void PolynomialConstraint<N>::getJacobian(
    const aikido::statespace::StateSpace::State* _s,
    Eigen::MatrixXd& _out) const
{
  Eigen::VectorXd derivCoeffs(mCoeffs.rows() - 1);
  for (int i = 0; i < derivCoeffs.rows(); i++)
    derivCoeffs(i) = mCoeffs(i + 1) * (i + 1);

  PolynomialConstraint derivPoly(derivCoeffs);

  Eigen::VectorXd out;
  derivPoly.getValue(_s, out);
  _out.resize(out.rows(), 1);
  _out.col(0) = out;
}

//==============================================================================
template <int N>
std::vector<aikido::constraint::ConstraintType>
PolynomialConstraint<N>::getConstraintTypes() const
{
  std::vector<aikido::constraint::ConstraintType> ctypes;
  ctypes.push_back(aikido::constraint::ConstraintType::EQUALITY);
  return ctypes;
}

//==============================================================================
template <int N>
aikido::statespace::ConstStateSpacePtr PolynomialConstraint<N>::getStateSpace()
    const
{
  return mStateSpace;
}
