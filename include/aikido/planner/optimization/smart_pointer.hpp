#ifndef AIKIDO_PLANNER_OPTIMIZATION_SMARTPOINTER_HPP_
#define AIKIDO_PLANNER_OPTIMIZATION_SMARTPOINTER_HPP_

#include "aikido/common/memory.hpp"

namespace aikido {
namespace planner {
namespace optimization {

AIKIDO_COMMON_DECLARE_SMART_POINTER(Function)
AIKIDO_COMMON_DECLARE_SMART_POINTER(CompositeFunction)

AIKIDO_COMMON_DECLARE_SMART_POINTER(Variable)
AIKIDO_COMMON_DECLARE_SMART_POINTER(CompositeVariable)
AIKIDO_COMMON_DECLARE_SMART_POINTER(TrajectoryVariable)

} // namespace optimization
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OPTIMIZATION_SMARTPOINTER_HPP_
