#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_H
#define AIKIDO_STATESPACE_REALVECTORSTATESPACE_H

#include "StateSpace.hpp"
#include "State.hpp"
#include "Jacobian.hpp"

namespace aikido {
namespace statespace {

/// Represents a n-dimensional real vector space.
class RealVectorStateSpace : public StateSpace
{
public:
  /// Point in a RealVectorStateSpace.
  class State : public StateSpace::State
  {
  public:
    explicit State(const Eigen::VectorXd& _x);

    Eigen::VectorXd mValue;
  };

  /// Constructs a RealVectorStateSpace with the given dimensionality.
  RealVectorStateSpace(int _dimension);

  // Documentation inherited.
  int getRepresentationDimension() const override;

  // Documentation inherited.
  void compose(
    const StateSpace::State& _state1, const StateSpace::State& _state2,
    StateSpace::State& _out) const override;

private:
  int mDimension;
};

} // namespace statespace
} // namespace aikido

#endif
