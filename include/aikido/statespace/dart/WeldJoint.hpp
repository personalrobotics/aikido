#ifndef AIKIDO_STATESPACE_DART_WELDJOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_WELDJOINTSTATESPACE_HPP_

#include "aikido/statespace/Rn.hpp"
#include "aikido/statespace/dart/JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c Weld for DART \c WeldJoint. This class treats the
/// joint's positions as a real vector space.
class WeldJoint
  : public R0
  , public JointStateSpace
  , public std::enable_shared_from_this<WeldJoint>
{
public:
  using R::State;

  /// Create a real vector state space for \c joint.
  ///
  /// \param joint joint to create a state space for
  explicit WeldJoint(const ::dart::dynamics::WeldJoint* joint);

  // Documentation inherited.
  void convertPositionsToState(
      const Eigen::VectorXd& positions,
      StateSpace::State* state) const override;

  // Documentation inherited.
  void convertStateToPositions(
      const StateSpace::State* state,
      Eigen::VectorXd& positions) const override;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_WELDJOINTSTATESPACE_HPP_
