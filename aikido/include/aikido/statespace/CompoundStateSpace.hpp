#ifndef AIKIDO_STATESPACE_COMPOUNDSTATESPACE_HPP_
#define AIKIDO_STATESPACE_COMPOUNDSTATESPACE_HPP_
#include <vector>
#include "StateSpace.hpp"
#include "ScopedState.hpp"

namespace aikido {
namespace statespace {

// Defined in detail/CompoundStateSpace.hpp
template <class>
class CompoundStateHandle;

/// Represents the Cartesian product of other <tt>StateSpace</tt>s.
class CompoundStateSpace
  : public std::enable_shared_from_this<CompoundStateSpace>
  , public virtual StateSpace
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
  /// \param subspaces vector of subspaces
  explicit CompoundStateSpace(std::vector<StateSpacePtr> _subspaces);

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c CompoundStateSpace::ScopedState
  ScopedState createState() const;

  /// Gets number of subspaces.
  ///
  /// \return number of subspaces
  size_t getNumStates() const;

  /// Gets subspace of type \c Space by at \c _index.
  ///
  /// \param _index in the range [ 0, \c getNumStates() ]
  /// \return \c StateSpace at \c _index
  template <class Space = StateSpace>
  std::shared_ptr<Space> getSubSpace(size_t _index) const;

  /// Gets substate of type \c Space::State from a CompoundState by index.
  ///
  /// \param _state state in this \c CompoundStateSpace
  /// \param _index in the range [ 0, \ getNumStates() ]
  /// \return \c State at \c _index
  template <class Space = StateSpace>
  typename Space::State *getSubState(StateSpace::State *_state,
                                     size_t _index) const;

  /// Gets substate of type \c Space::State from a CompoundState by index. This
  /// is an overload for when \c _state is \c const.
  ///
  /// \param _state state in this \c CompoundStateSpace
  /// \param _index in the range [ 0, \ getNumStates() ]
  template <class Space = StateSpace>
  const typename Space::State *getSubState(const StateSpace::State *_state,
                                           size_t _index) const;

  /// Gets substate of type \c Space::State from a CompoundState by index and
  /// wraps it in a \c Space::StateHandle helper class.
  ///
  /// \param _state state in this \c CompoundStateSpace
  /// \param _index in the range [ 0, \ getNumStates() ]
  template <class Space = StateSpace>
  typename Space::StateHandle getSubStateHandle(StateSpace::State *_state,
                                                size_t _index) const;

  /// Gets substate of type \c Space::State from a CompoundState by index and
  /// wraps it in a \c Space::ConstStateHandle helper class. This is an
  /// overload for when \c _state is \c const.
  ///
  /// \param _state state in this \c CompoundStateSpace
  /// \param _index in the range [ 0, \ getNumStates() ]
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
  void copyState(StateSpace::State* _destination,
                 const StateSpace::State* _source) const override;

  // Documentation inherited.
  void expMap(
    const Eigen::VectorXd& _tangent, StateSpace::State* _out) const override;

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

#endif // AIKIDO_STATESPACE_COMPOUNDSTATESPACE_HPP_
