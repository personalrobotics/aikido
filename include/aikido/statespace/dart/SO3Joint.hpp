#ifndef AIKIDO_STATESPACE_DART_SO3JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_SO3JOINTSTATESPACE_HPP_

#include "aikido/statespace/SO3.hpp"
#include "aikido/statespace/dart/JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c SO3 for a DART \c BallJoint. This class does not support
/// position limits.
class SO3Joint : public SO3,
                 public JointStateSpace,
                 public std::enable_shared_from_this<SO3Joint>
{
public:
  using SO3::State;

  /// Creates a state space for a \c BallJoint. This class does not support
  /// position limits.
  ///
  /// \param joint joint to create a state space for
  explicit SO3Joint(const ::dart::dynamics::BallJoint* joint);

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

#endif // ifndef AIKIDO_STATESPACE_SO3JOINTSTATESPACE_HPP_
