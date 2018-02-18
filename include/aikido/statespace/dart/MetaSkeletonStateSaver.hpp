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
  /// Options to specify what MetaSkeletonStateSaver should save.
  enum Options
  {
    POSITIONS = 1 << 0,
    POSITION_LIMITS = 1 << 1,
  };

  /// Construct a MetaSkeletonStateSaver and save the current state of the \c
  /// MetaSkeleton. This state will be restored when MetaSkeletonStateSaver is
  /// destructed.
  ///
  /// \param metaskeleton MetaSkeleton to save/restore
  /// \param options Options to specify what should be saved
  explicit MetaSkeletonStateSaver(
      ::dart::dynamics::MetaSkeletonPtr metaskeleton,
      int options = POSITIONS | POSITION_LIMITS);

  virtual ~MetaSkeletonStateSaver();

  // MetaSkeletonStateSaver is uncopyable, must use std::move
  MetaSkeletonStateSaver(const MetaSkeletonStateSaver&) = delete;
  MetaSkeletonStateSaver& operator=(const MetaSkeletonStateSaver&) = delete;

  MetaSkeletonStateSaver(MetaSkeletonStateSaver&&) = default;
  MetaSkeletonStateSaver& operator=(MetaSkeletonStateSaver&&) = default;

private:
  /// MetaSkeleton to save the state of
  ::dart::dynamics::MetaSkeletonPtr mMetaSkeleton;

  /// Options to specify what should be saved
  int mOptions;

  /// Saved positions
  Eigen::VectorXd mPositions;

  /// Saved position lower limits
  Eigen::VectorXd mPositionLowerLimits;

  /// Saved position upper limits
  Eigen::VectorXd mPositionUpperLimits;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_DART_METASKELETONSTATESAVER_HPP_
