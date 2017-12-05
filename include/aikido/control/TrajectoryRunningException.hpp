#ifndef AIKIDO_CONTROL_TRAJECTORYRUNNINGEXCEPTION_HPP_
#define AIKIDO_CONTROL_TRAJECTORYRUNNINGEXCEPTION_HPP_

#include <stdexcept>

namespace aikido {
namespace control {

class TrajectoryRunningException : public std::runtime_error
{
public:
  TrajectoryRunningException();

  virtual ~TrajectoryRunningException() = default;
};

} // namespace control
} // namespace aikido

#endif // AIKIDO_CONTROL_TRAJECTORYRUNNINGEXCEPTION_HPP_
