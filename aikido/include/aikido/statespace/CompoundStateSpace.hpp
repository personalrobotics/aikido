#ifndef AIKIDO_STATESPACE_COMPOUNDSTATESPACE_H
#define AIKIDO_STATESPACE_COMPOUNDSTATESPACE_H
#include <vector>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido
{
namespace statespace
{
// Defined in detail/CompoundStateSpace.hpp
template <class>
class CompoundStateHandle;

/// Represents the Cartesian product of other StateSpaces.
class CompoundStateSpace
    : public std::enable_shared_from_this<CompoundStateSpace>,
      public virtual StateSpace
{
public:
  /// A tuple of states where the i-th state is from the i-th subspace.
  class State : public StateSpace::State
  {
  protected:
    State() = default;
    ~State() = default;

    friend class CompoundStateSpace;
  };

  using StateHandle = CompoundStateHandle<State>;
  using StateHandleConst = CompoundStateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  /// Construct the Cartesian product of a vector of subspaces.
  explicit CompoundStateSpace(const std::vector<StateSpacePtr> &_subspaces);

  /// Helper function to create a ScopedState.
  ScopedState createState() const;

  /// Gets number of subspaces.
  size_t getNumStates() const;

  /// Gets subspace by index.
  template <class Space = StateSpace>
  const Space *getSubSpace(size_t _index) const;

  /// Gets state of type by subspace index.
  template <class Space = StateSpace>
  typename Space::State *getSubState(StateSpace::State *_state,
                                     size_t _index) const;

  /// Gets state of type by subspace index.
  template <class Space = StateSpace>
  const typename Space::State *getSubState(const StateSpace::State *_state,
                                           size_t _index) const;

  /// Gets state handle of type by subspace index.
  template <class Space = StateSpace>
  typename Space::StateHandle getSubStateHandle(StateSpace::State *_state,
                                                size_t _index) const;

  /// Gets state handle of type by subspace index.
  template <class Space = StateSpace>
  typename Space::StateHandleConst getSubStateHandle(
      const StateSpace::State *_state, size_t _index) const;

  // Documentation inherited.
  size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State *allocateStateInBuffer(void *_buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State *_state) const override;

  // Documentation inherited.
  void compose(const StateSpace::State *_state1,
               const StateSpace::State *_state2,
               StateSpace::State *_out) const override;

  // Documentation inherited
  void getIdentity(StateSpace::State *_out) const override;

  // Documentation inherited
  void getInverse(const StateSpace::State *_in,
                  StateSpace::State *_out) const override;

  // Documentation inherited
  unsigned int getDimension() const override;

  // Documentation inherited
  double getMaximumExtent() const override;

  // Documentation inherited
  double getMeasure() const override;

  // Documentation inherited
  void copyState(StateSpace::State *_destination,
                 const StateSpace::State *_source) const override;

  // Documentation inherited
  double distance(const StateSpace::State *_state1,
                  const StateSpace::State *_state2) const override;

  // Documentation inherited
  bool equalStates(const StateSpace::State *_state1,
                   const StateSpace::State *_state2) const override;

  // Documentation inherited
  void interpolate(const StateSpace::State *_from, const StateSpace::State *_to,
                   const double _t, StateSpace::State *_State) const override;

  // Documentation inherited.
  void expMap(const Eigen::VectorXd &_tangent,
              StateSpace::State *_out) const override;

  // Documentation inherited
  void logMap(const StateSpace::State *_in,
              Eigen::VectorXd &_tangent) const override;

private:
  std::vector<StateSpacePtr> mSubspaces;
  std::vector<std::size_t> mOffsets;
  size_t mSizeInBytes;
};

}  // namespace statespace
}  // namespace aikido

#include "detail/CompoundStateSpace.hpp"

#endif
