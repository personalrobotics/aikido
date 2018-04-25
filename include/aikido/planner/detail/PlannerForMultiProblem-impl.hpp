#ifndef AIKIDO_PLANNER_DETAIL_CONCRETEPLANNER_IMPL_HPP_
#define AIKIDO_PLANNER_DETAIL_CONCRETEPLANNER_IMPL_HPP_

#include "aikido/planner/PlannerForMultiProblem.hpp"

namespace aikido {
namespace planner {

//==============================================================================
template <class ProblemT, typename T, typename R, typename... Args>
void PlannerForMultiProblem::registerPlanningFunction(R (T::*func)(Args...))
{
  auto& map = getPlanningFunctionMap();
  auto bindedFunc = std::bind(
      func,
      static_cast<T*>(this),
      std::placeholders::_1,  // problem
      std::placeholders::_2); // result

  map.insert(std::make_pair(ProblemT::getStaticType(), bindedFunc));
}

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DETAIL_CONCRETEPLANNER_IMPL_HPP_
