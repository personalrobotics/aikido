#ifndef AIKIDO_CONTROL_TRAJECTORYRESULT_HPP_
#define AIKIDO_CONTROL_TRAJECTORYRESULT_HPP_

#include <memory>

namespace aikido {
namespace control {

class TrajectoryResult
{
public:
  virtual ~TrajectoryResult() = default;
};

using TrajectoryResultPtr = std::shared_ptr<TrajectoryResult>;

} // control
} // aikido

#endif
