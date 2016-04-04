#ifndef AIKIDO_STATESPACE_REALVECTORSTATESPACE_H
#define AIKIDO_STATESPACE_REALVECTORSTATESPACE_H
#include <Eigen/Core>
#include "StateSpace.hpp"

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
    ~State() = default;

    /// Get value.
    const Eigen::VectorXd& getValue() const;

    /// Set value.
    void setValue(const Eigen::VectorXd& _x);

  private:
    Eigen::VectorXd mValue;

    friend class RealVectorStateSpace;
  };

  /// Constructs a RealVectorStateSpace with the given dimensionality.
  explicit RealVectorStateSpace(int _dimension);

  // Documentation inherited.
  size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateState() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* _buffer) const;

  // Documentation inherited.
  void freeState(StateSpace::State* _state) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State* _state) const override;

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
