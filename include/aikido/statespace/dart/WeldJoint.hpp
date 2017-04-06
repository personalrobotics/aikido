#ifndef AIKIDO_STATESPACE_DART_WELDJOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_WELDJOINTSTATESPACE_HPP_
#include "../Rn.hpp"
#include "JointStateSpace.hpp"

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
  using Rn::State;

  /// Create a real vector state space for \c _joint.
  ///
  /// \param _joint joint to create a state space for
  explicit WeldJoint(::dart::dynamics::WeldJoint* _joint);

  // Documentation inherited.
  void convertPositionsToState(
    const Eigen::VectorXd& _positions,
    StateSpace::State* _state) const override;

  // Documentation inherited.
  void convertStateToPositions(
    const StateSpace::State* _state,
    Eigen::VectorXd& _positions) const override;
};

} // namespace dart
} // namespace statespace
} // namespace aikido

#endif // ifndef AIKIDO_STATESPACE_WELDJOINTSTATESPACE_HPP_
