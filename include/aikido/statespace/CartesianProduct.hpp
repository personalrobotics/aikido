#ifndef AIKIDO_STATESPACE_COMPOUNDSTATESPACE_HPP_
#define AIKIDO_STATESPACE_COMPOUNDSTATESPACE_HPP_
#include <vector>

#include "aikido/statespace/ScopedState.hpp"
#include "aikido/statespace/StateSpace.hpp"

namespace aikido {
namespace statespace {

AIKIDO_DECLARE_POINTERS(CartesianProduct)

// Defined in detail/CartesianProduct.hpp
template <class>
class CompoundStateHandle;

/// Represents the Cartesian product of other <tt>StateSpace</tt>s.
class CartesianProduct
  : public std::enable_shared_from_this<CartesianProduct>
  , public virtual StateSpace
{
public:
  class State;

  using StateHandle = CompoundStateHandle<State>;
  using StateHandleConst = CompoundStateHandle<const State>;

  using ScopedState = statespace::ScopedState<StateHandle>;
  using ScopedStateConst = statespace::ScopedState<StateHandleConst>;

  using StateSpace::compose;

  /// Construct the Cartesian product of a vector of subspaces.
  /// \param _subspaces vector of subspaces
  explicit CartesianProduct(std::vector<ConstStateSpacePtr> _subspaces);

  /// Helper function to create a \c ScopedState.
  ///
  /// \return new \c ScopedState
  ScopedState createState() const;

  /// Creates an identical clone of \c stateIn.
  ScopedState cloneState(const StateSpace::State* stateIn) const;

  /// Gets number of subspaces.
  ///
  /// \return number of subspaces
  std::size_t getNumSubspaces() const;

  /// Gets subspace of type \c Space by at \c _index.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _index in the range [ 0, \c getNumSubspaces() ]
  /// \return subspace at \c _index
  template <class Space = StateSpace>
  std::shared_ptr<const Space> getSubspace(std::size_t _index) const;

  /// Gets substate of type \c Space::State from a CompoundState by index.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _state state in this \c CartesianProduct
  /// \param _index in the range [ 0, \c getNumSubspaces() ]
  /// \return state at \c _index
  template <class Space = StateSpace>
  typename Space::State* getSubState(State* _state, std::size_t _index) const;

  /// Gets substate of type \c Space::State from a CompoundState by index. This
  /// is an overload for when \c _state is \c const.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _state state in this \c CartesianProduct
  /// \param _index in the range [ 0, \c getNumSubspaces() ]
  /// \return state at \c _index
  template <class Space = StateSpace>
  const typename Space::State* getSubState(
      const State* _state, std::size_t _index) const;

  /// Gets substate of type \c Space::State from a CompoundState by index and
  /// wraps it in a \c Space::StateHandle helper class.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _state state in this \c CartesianProduct
  /// \param _index in the range [ 0, \c getNumSubspaces() ]
  /// \return state at \c _index
  template <class Space = StateSpace>
  typename Space::StateHandle getSubStateHandle(
      State* _state, std::size_t _index) const;

  /// Gets substate of type \c Space::State from a CompoundState by index and
  /// wraps it in a \c Space::ConstStateHandle helper class. This is an
  /// overload for when \c _state is \c const.
  ///
  /// \tparam Space type of \c StateSpace for subspace \c _index
  /// \param _state state in this \c CartesianProduct
  /// \param _index in the range [ 0, \ getNumSubspaces() ]
  /// \return state at \c _index
  template <class Space = StateSpace>
  typename Space::StateHandleConst getSubStateHandle(
      const State* _state, std::size_t _index) const;

  // Documentation inherited.
  std::size_t getStateSizeInBytes() const override;

  // Documentation inherited.
  StateSpace::State* allocateStateInBuffer(void* _buffer) const override;

  // Documentation inherited.
  void freeStateInBuffer(StateSpace::State* _state) const override;

  // Documentation inherited.
  void compose(
      const StateSpace::State* _state1,
      const StateSpace::State* _state2,
      StateSpace::State* _out) const override;

  // Documentation inherited
  void getIdentity(StateSpace::State* _state) const override;

  // Documentation inherited
  void getInverse(
      const StateSpace::State* _in, StateSpace::State* _out) const override;

  // Documentation inherited
  std::size_t getDimension() const override;

  // Documentation inherited
  void copyState(
      const StateSpace::State* _source,
      StateSpace::State* _destination) const override;

  /// Exponential mapping of Lie algebra element to a Lie group element. The
  /// tangent space is parameterized by stacking the tangent vector of each
  /// subspace in the order the subspaces are listed in.
  ///
  /// \param _tangent element of the tangent space
  /// \param[out] _out corresponding element of the Lie group
  void expMap(
      const Eigen::VectorXd& _tangent, StateSpace::State* _out) const override;

  /// Log mapping of Lie group element to a Lie algebra element. The tangent
  /// space is parameterized by stacking the tangent vector of each subspace
  /// in the order the subspaces are listed in.
  ///
  /// \param _in element of this Lie group
  /// \param[out] _tangent corresponding element of the tangent space
  void logMap(
      const StateSpace::State* _in, Eigen::VectorXd& _tangent) const override;

  /// Print the contents of each substate contained in the state
  /// as a list with each substate enclosed in brackets and including its
  /// index
  /// Format: [0: ...] [1: ...] ... [n: ...]
  void print(const StateSpace::State* _state, std::ostream& _os) const override;

private:
  std::vector<ConstStateSpacePtr> mSubspaces;
  std::vector<std::size_t> mOffsets;
  std::size_t mSizeInBytes;
};

/// A tuple of states where the i-th state is from the i-th subspace.
class CartesianProduct::State : public StateSpace::State
{
protected:
  friend class CartesianProduct;

  State() = default;

  ~State() = default;
};

} // namespace statespace
} // namespace aikido

#include "detail/CartesianProduct-impl.hpp"

#endif // AIKIDO_STATESPACE_COMPOUNDSTATESPACE_HPP_
