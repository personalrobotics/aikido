#ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_SE3JOINTSTATESPACE_HPP_
#include "../SE3StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c SE3StateSpace for a DART \c FreeJoint. This class does not support
/// position limits on the three rotational <tt>DegreeOfFreedom</tt>s.
class SE3JointStateSpace
  : public SE3StateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<SE3JointStateSpace>
{
public:
  using SE3StateSpace::State;

  /// Creates a state space for a \c FreeJoint. This class does not support
  /// position limits on the rotational <tt>DegreeOfFreedom</tt>s.
  ///
  /// \param _joint joint to create a state space for
  explicit SE3JointStateSpace(::dart::dynamics::FreeJoint* _joint);

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

#endif // ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_HPP_
