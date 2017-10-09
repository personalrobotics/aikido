#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace trajectory {

//==============================================================================
void Trajectory::setParameters(const Eigen::VectorXd& parameters)
{
  if (static_cast<std::size_t>(parameters.size())
      != getStateSpace()->getDimension() * getNumParameters())
  {
    throw std::invalid_argument("Mismatch in argument length.");
  }

  const Eigen::Map<const Eigen::MatrixXd> mat(
      parameters.data(), getStateSpace()->getDimension(), getNumParameters());

  setParameters(static_cast<Eigen::MatrixXd>(mat));
}

} // namespace trajectory
} // namespace aikido
