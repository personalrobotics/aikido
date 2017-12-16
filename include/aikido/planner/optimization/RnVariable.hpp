#ifndef AIKIDO_PLANNER_OPTIMIZATION_RNVARIABLE_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_RNVARIABLE_HPP_

#include <memory>
#include "aikido/planner/optimization/Variable.hpp"
#include "aikido/statespace/Rn.hpp"

namespace aikido {
namespace planner {
namespace optimization {

template <int N>
class RVariable : public Variable
{
public:
  /// Clone
  UniqueVariablePtr clone() const override;
  // TODO(JS): Change this to unique_ptr

  /// Returns the dimension of optimization variables.
  std::size_t getDimension() const override;

  /// Sets the optimization variables.
  void setValue(const Eigen::VectorXd& value) override;

  /// Returns the optimization variables.
  Eigen::VectorXd getValue() const override;

public:
  std::size_t mDimension;

  Eigen::Matrix<double, N, 1> mValue;
};

using R0Variable = RVariable<0>;
using R1Variable = RVariable<1>;
using R2Variable = RVariable<2>;
using R3Variable = RVariable<3>;
using R4Variable = RVariable<4>;
using R6Variable = RVariable<6>;
// using RnVariable = RVariable<Eigen::Dynamic>;

//==============================================================================
template <int N>
UniqueVariablePtr RVariable<N>::clone() const
{
  return dart::common::make_unique<RVariable<N>>();
}

//==============================================================================
template <int N>
std::size_t RVariable<N>::getDimension() const
{
  if (N == Eigen::Dynamic)
    return mDimension;
  else
    return N;
}

//==============================================================================
template <int N>
void RVariable<N>::setValue(const Eigen::VectorXd& value)
{
  mValue = value;
}

//==============================================================================
template <int N>
Eigen::VectorXd RVariable<N>::getValue() const
{
  return mValue;
}

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_RNVARIABLE_HPP_
