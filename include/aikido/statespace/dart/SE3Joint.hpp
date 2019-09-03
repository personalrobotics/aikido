#ifndef AIKIDO_STATESPACE_DART_SE3JOINTSTATESPACE_HPP_
#define AIKIDO_STATESPACE_DART_SE3JOINTSTATESPACE_HPP_

#include "aikido/statespace/SE3.hpp"
#include "aikido/statespace/dart/JointStateSpace.hpp"

namespace aikido {
namespace statespace {
namespace dart {

/// \c SE3 for a DART \c FreeJoint. This class does not support
/// position limits on the three rotational <tt>DegreeOfFreedom</tt>s.
class SE3Joint
  : public SE3
  , public JointStateSpace
  , public std::enable_shared_from_this<SE3Joint>
{
public:
  using SE3::State;

  /// Creates a state space for a \c FreeJoint. This class does not support
  /// position limits on the rotational <tt>DegreeOfFreedom</tt>s.
  ///
  /// \param joint joint to create a state space for
  explicit SE3Joint(const ::dart::dynamics::FreeJoint* joint);

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

#endif // ifndef AIKIDO_STATESPACE_SE3JOINTSTATESPACE_HPP_
