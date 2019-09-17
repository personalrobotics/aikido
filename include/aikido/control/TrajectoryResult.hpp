#ifndef AIKIDO_CONTROL_TRAJECTORYRESULT_HPP_
#define AIKIDO_CONTROL_TRAJECTORYRESULT_HPP_

#include <memory>

#include "aikido/common/pointers.hpp"

namespace aikido {
namespace control {

AIKIDO_DECLARE_POINTERS(TrajectoryResult)

class TrajectoryResult
{
public:
  virtual ~TrajectoryResult() = default;
};

} // namespace control
} // namespace aikido

#endif
