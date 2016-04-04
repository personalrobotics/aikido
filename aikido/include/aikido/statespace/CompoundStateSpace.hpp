#ifndef AIKIDO_STATESPACE_COMPOUNDSTATESPACE_H
#define AIKIDO_STATESPACE_COMPOUNDSTATESPACE_H
#include <vector>
#include "StateSpace.hpp"

namespace aikido {
namespace statespace {

/// Represents the Cartesian product of other StateSpaces.
class CompoundStateSpace : public StateSpace
{
public:
  /// A tuple of states where the i-th state is from the i-th subspace.
  class State : public StateSpace::State
  {
  public:
    /// Create a CompoundState from a vector of states. This takes ownership of
    /// the the pointers in vector.
    explicit State(const std::vector<StateSpace::State*>& _states);

    virtual ~State();

    // Disable copying and assignment because this class manages memory.
    State(const State& _other) = delete;
    State(State&& _other) = delete;
    State& operator =(const State& _other) = delete;
    State& operator =(State&& _other) = delete;

    /// Gets number of subspaces.
    size_t getNumStates() const;

    /// Gets state by subspace index.
    StateSpace::State& getState(size_t _index);

    /// Gets state by subspace index.
    const StateSpace::State& getState(size_t _index) const;

    /// Gets the vector of underlying states.
    const std::vector<StateSpace::State*>& getStates();

    /// Gets the vector of underlying states.
    std::vector<const StateSpace::State*> getStates() const;

  private:
    std::vector<StateSpace::State*> mValue;

    friend class CompoundStateSpace;
  };

  /// Construct the Cartesian product of a vector of subspaces.
  explicit CompoundStateSpace(const std::vector<StateSpacePtr>& _subspaces);
   
  // Documentation inherited.
  void compose(
    const StateSpace::State& _state1, const StateSpace::State& _state2,
    StateSpace::State& _out) const override;

  // Documentation inherited.
  int getRepresentationDimension() const override;

private:
  std::vector<StateSpacePtr> mSubspaces;
  int mRepresentationDimension;
};

} // namespace statespace
} // namespace aikido

#endif
