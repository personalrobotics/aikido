#ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
#include "../SE3StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// Wrap a 6-DOF joint in a SE3StateSpace.
class SE3JointStateSpace
  : public SE3StateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<SE3JointStateSpace>
{
public:
  using SE3StateSpace::State;

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

#endif // ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_H_
