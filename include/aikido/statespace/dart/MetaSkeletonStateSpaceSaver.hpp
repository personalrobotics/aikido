#ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESPACESAVER_HPP_
#define AIKIDO_STATESPACE_DART_METASKELETONSTATESPACESAVER_HPP_
#include "MetaSkeletonStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// RAII class to save and restore a MetaSkeletonStateSpace's state.
/// FIXME: currently only saves position.
class MetaSkeletonStateSpaceSaver
{
public:

  /// Construct a MetaSkeletonStateSpaceSaver and save the current state of the
  /// \c MetaSkeletonStateSpace. This state will be restored when
  /// MetaSkeletonStateSpaceSaver is destructed.
  ///
  /// \param _space MetaSkeletonStateSpace to save/restore
  explicit MetaSkeletonStateSpaceSaver(MetaSkeletonStateSpacePtr _space);

  virtual ~MetaSkeletonStateSpaceSaver();

  // MetaSkeletonStateSpaceSaver is uncopyable, must use std::move
  MetaSkeletonStateSpaceSaver(const MetaSkeletonStateSpaceSaver&) = delete;
  MetaSkeletonStateSpaceSaver& operator =(const MetaSkeletonStateSpaceSaver&) = delete;

  MetaSkeletonStateSpaceSaver(MetaSkeletonStateSpaceSaver&&) = default;
  MetaSkeletonStateSpaceSaver& operator =(MetaSkeletonStateSpaceSaver&&) = default;

private:
  MetaSkeletonStateSpacePtr mSpace;
  Eigen::VectorXd mPositions;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#include "detail/MetaSkeletonStateSpaceSaver-impl.hpp"

#endif // ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESPACESAVER_HPP_
