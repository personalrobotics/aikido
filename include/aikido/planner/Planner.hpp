#ifndef AIKIDO_PLANNER_PLANNER_HPP_
#define AIKIDO_PLANNER_PLANNER_HPP_

#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include "aikido/planner/Problem.hpp"
#include "aikido/trajectory/Trajectory.hpp"

namespace aikido {
namespace planner {

/// Base class for a meta-planner, to be implemented.
/// [Gilwoo] Created a strawman for Robot class
class Planner
{
public:
  virtual bool canSolve(const Problem* problem);

  virtual trajectory::TrajectoryPtr solve(
      const Problem* problem, Problem::Result* result = nullptr);

protected:
  using PlanningFunction = std::function<trajectory::TrajectoryPtr(
      const Problem*, Problem::Result*)>;
  using PlanningFunctionMap = std::map<std::string, PlanningFunction>;

  virtual PlanningFunctionMap& getPlanningFunctionMap() = 0;

  template <class ProblemT, typename T, typename R, typename... Args>
  void registerPlanningFunction(R (T::*func)(Args...))
  {
    //    static_assert(std::is_same<typename ProblemT::ReturnTrajectoryType,
    //                               typename
    //                               std::pointer_traits<R>::element_type>::value,
    //                               "TODO: error message");

    auto& map = getPlanningFunctionMap();
    auto func2 = std::bind(
        func,
        static_cast<T*>(this),
        std::placeholders::_1,
        std::placeholders::_2);

    map.insert(std::make_pair(ProblemT::getStaticName(), func2));
  }
};

} // namespace planner
} // namespace aikido

#endif
