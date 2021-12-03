#ifndef AIKIDO_ROBOT_IK_GRADIENTDESCENTIK_HPP_
#define AIKIDO_ROBOT_IK_GRADIENTDESCENTIK_HPP_

namespace aikido {
namespace robot {
namespace ik {

/// Optimization-based IK solver that uses gradient descent from random seed
// configurations.
class GradientDescentIk : public InverseKinematics
{
public:

  GradientDescentIk();

  // Documentation inherited.
  std::vector<Eigen::VectorXd> getSolutions(
    const Eigen::Isometry3d& targetPose, const size_t maxSolutions) override;

  /// Destructor.
  virtual ~GradientDescentIk() = default;
};

} // namespace ik
} // namespace robot
} // namespace aikido

#endif // AIKIDO_ROBOT_IK_GRADIENTDESCENTIK_HPP_
