#ifndef AIKIDO_STATESPACE_STATESPACESTATESAVER_HPP_
#define AIKIDO_STATESPACE_STATESPACESTATESAVER_HPP_

namespace aikido {
namespace statespace {

/// RAII class to save and restore a MetaSkeletonStateSpace's state.
/// FIXME: currently only saves position.
class StateSpaceStateSaver
{
public:
  using MetaSkeletonStateSpacePtr = dart::MetaSkeletonStateSpacePtr;

  /// Construct a StateSpaceStateSaver and save the current state of the
  /// \c MetaSkeletonStateSpace. This state will be restored when
  /// StateSpaceStateSaver is destructed.
  ///
  /// \param _space MetaSkeletonStateSpace to save/restore
  explicit StateSpaceStateSaver(MetaSkeletonStateSpacePtr _space);

  virtual ~StateSpaceStateSaver();

  // StateSpaceStateSaver is uncopyable, must use std::move
  StateSpaceStateSaver(const StateSpaceStateSaver&) = delete;
  StateSpaceStateSaver& operator =(const StateSpaceStateSaver&) = delete;

  StateSpaceStateSaver(StateSpaceStateSaver&&) = default;
  StateSpaceStateSaver& operator =(StateSpaceStateSaver&&) = default;

private:
  MetaSkeletonStateSpacePtr mSpace;
  Eigen::VectorXd mPositions;
};

} // namespace statespace
} // namespace aikido

#include "detail/StateSpaceStateSaver-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_STATESPACESTATESAVER_HPP_
