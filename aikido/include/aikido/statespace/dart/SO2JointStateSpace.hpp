#ifndef AIKIDO_STATESPACE_DART_SO2JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_SO2JOINTSTATESPACE_HPP_
#include "../SO2.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c SO2 for a DART \c SingleDofJoint. This class does not support
/// position limits.
class SO2JointStateSpace
  : public SO2
  , public JointStateSpace
  , public std::enable_shared_from_this<SO2JointStateSpace>
{
public:
  using SO2::State;

  /// Creates a state space for a \c FreeJoint. This class does not support
  /// position limits.
  ///
  /// \param _joint joint to create a state space for
  explicit SO2JointStateSpace(::dart::dynamics::SingleDofJoint* _joint);

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

#endif // ifndef AIKIDO_STATESPACE_SO2JOINTSTATESPACE_HPP_
