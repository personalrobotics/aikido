#ifndef AIKIDO_PLANNER_DETAIL_PLANNER_IMPL_HPP_
#define AIKIDO_PLANNER_DETAIL_PLANNER_IMPL_HPP_

#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
template <class ProblemT, typename T, typename R, typename... Args>
void Planner::registerPlanningFunction(R (T::*func)(Args...))
{
  static_assert(
      std::is_same<typename ProblemT::ReturnTrajectoryType,
                   typename std::pointer_traits<R>::element_type>::value,
      "TODO: error message");
  // TODO(JS): Not sure if returning trajectory type should be tied to problem
  // or planner. If it's not, this check and ReturnTrajectoryType should be
  // removed.

  auto& map = getPlanningFunctionMap();
  auto bindedFunc = std::bind(
      func,
      static_cast<T*>(this),
      std::placeholders::_1,  // problem
      std::placeholders::_2); // result

  map.insert(std::make_pair(ProblemT::getStaticName(), bindedFunc));
}

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DETAIL_PLANNER_IMPL_HPP_
