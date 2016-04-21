#ifndef AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
#define AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
#include "../SO2StateSpace.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// Wrap a single DOF joint in a SO2StateSpace.
class SO2JointStateSpace
  : public SO2StateSpace
  , public JointStateSpace
  , public std::enable_shared_from_this<SO2JointStateSpace>
{
public:
  using SO2StateSpace::State;

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

#endif // ifndef AIKIDO_STATESPACE_SO2JOINTSTATESPACE_H_
