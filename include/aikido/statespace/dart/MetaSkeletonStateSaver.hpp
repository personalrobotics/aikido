#ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESAVER_HPP_
#define AIKIDO_STATESPACE_DART_METASKELETONSTATESAVER_HPP_

#include <dart/dynamics/MetaSkeleton.hpp>

namespace aikido {
namespace statespace {
namespace dart {

/// RAII class to save and restore a MetaSkeleton's state.
/// FIXME: currently only saves position and joint limits.
class MetaSkeletonStateSaver
{
public:
  /// Construct a MetaSkeletonStateSaver and save the current state of the \c
  /// MetaSkeleton. This state will be restored when MetaSkeletonStateSaver is
  /// destructed.
  ///
  /// \param _metaskeleton MetaSkeleton to save/restore
  explicit MetaSkeletonStateSaver(
      ::dart::dynamics::MetaSkeletonPtr _metaskeleton);

  virtual ~MetaSkeletonStateSaver();

  // MetaSkeletonStateSaver is uncopyable, must use std::move
  MetaSkeletonStateSaver(const MetaSkeletonStateSaver&) = delete;
  MetaSkeletonStateSaver& operator=(const MetaSkeletonStateSaver&) = delete;

  MetaSkeletonStateSaver(MetaSkeletonStateSaver&&) = default;
  MetaSkeletonStateSaver& operator=(MetaSkeletonStateSaver&&) = default;

private:
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;
  Eigen::VectorXd mPositions;
  Eigen::VectorXd mPositionLowerLimits;
  Eigen::VectorXd mPositionUpperLimits;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESAVER_HPP_
