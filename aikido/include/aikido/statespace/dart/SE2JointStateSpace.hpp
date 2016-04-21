#ifndef AIKIDO_STATESPACE_SE2JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SE2JOINTSTATESPACE_H_
#include "../SE2StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// Wrap a PlanarJoint in a SE2StateSpace.
class SE2JointStateSpace
  : public SE2StateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<SE2JointStateSpace>
{
public:
  using SE2StateSpace::State;
  using SE2StateSpace::Isometry2d;

  explicit SE2JointStateSpace(::dart::dynamics::PlanarJoint* _joint);

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

#endif // ifndef AIKIDO_STATESPACE_SE2JOINTSTATESPACE_H_
