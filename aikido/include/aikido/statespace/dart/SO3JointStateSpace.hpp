#ifndef AIKIDO_STATESPACE_SO3JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_SO3JOINTSTATESPACE_HPP_
#include "../SO3StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c SO3StateSpace for a DART \c BallJoint. This class does not support
/// position limits.
class SO3JointStateSpace
  : public SO3StateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<SO3JointStateSpace>
{
public:
  using SO3StateSpace::State;

  /// Creates a state space for a \c BallJoint. This class does not support
  /// position limits.
  ///
  /// \param _joint joint to create a state space for
  explicit SO3JointStateSpace(::dart::dynamics::BallJoint* _joint);

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

#endif // ifndef AIKIDO_STATESPACE_SO3JOINTSTATESPACE_HPP_
