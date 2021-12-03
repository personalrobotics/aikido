#include "aikido/robot/GradientDescentIk.hpp"

namespace aikido {
namespace robot {

//==============================================================================
GradientDescentIk::GradientDescentIk()
{
  // TODO.
}

//==============================================================================
std::vector<Eigen::VectorXd> GradientDescentIk::getSolutions(
    const Eigen::Isometry3d& targetPose,
    const size_t maxSolutions
) const {
  // TODO.
}

} // namespace robot
} // namespace aikido