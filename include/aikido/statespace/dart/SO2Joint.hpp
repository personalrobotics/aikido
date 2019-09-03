#ifndef AIKIDO_STATESPACE_DART_SO2JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_SO2JOINTSTATESPACE_HPP_

#include "aikido/statespace/SO2.hpp"
#include "aikido/statespace/dart/JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c SO2 for a DART \c SingleDofJoint. This class does not support
/// position limits.
class SO2Joint
  : public SO2
  , public JointStateSpace
  , public std::enable_shared_from_this<SO2Joint>
{
public:
  using SO2::State;

  /// Creates a state space for a \c FreeJoint. This class does not support
  /// position limits.
  ///
  /// \param joint joint to create a state space for
  explicit SO2Joint(
      const ::dart::dynamics::GenericJoint<::dart::math::R1Space>* joint);

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

#endif // ifndef AIKIDO_STATESPACE_SO2JOINTSTATESPACE_HPP_
