#ifndef AIKIDO_ROBOT_IK_INVERSEKINEMATICS_HPP_
#define AIKIDO_ROBOT_IK_INVERSEKINEMATICS_HPP_

namespace aikido {
namespace robot {

/// Interface for IK solvers used with the Robot class.
class InverseKinematics
{
public:
  /// Returns a set of IK solutions for the given pose. Note that *at most*
  /// \c maxSolutions solutions are returned, but fewer may be (and possibly
  /// none if solving fails).
  ///
  /// \param targetPose End-effector pose to solve inverse kinematics for.
  /// \param maxSolutions Upper limit on how many solutions to find and return.
  /// \return solution configurations that reach the target pose.
  virtual std::vector<Eigen::VectorXd> getSolutions(
      const Eigen::Isometry3d& targetPose, const size_t maxSolutions) const = 0;
};

} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_IK_INVERSEKINEMATICS_HPP_
