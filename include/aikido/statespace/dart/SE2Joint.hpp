#ifndef AIKIDO_STATESPACE_DART_SE2JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_SE2JOINTSTATESPACE_HPP_
#include "../SE2.hpp"
#include "JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c SEStateSpace for a DART \c PlanarJoint This class does not support
/// position limits on the rotational \c DegreeOfFreedom.
class SE2Joint : public SE2,
                 public JointStateSpace,
                 public std::enable_shared_from_this<SE2Joint>
{
public:
  using SE2::State;
  using SE2::Isometry2d;

  /// Creates a state space for a \c PlanarJoint. This class does not support
  /// position limits on the rotational \c DegreeOfFreedom.
  ///
  /// \param _joint joint to create a state space for
  explicit SE2Joint(const ::dart::dynamics::PlanarJoint* joint);

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

#endif // ifndef AIKIDO_STATESPACE_SE2JOINTSTATESPACE_HPP_
