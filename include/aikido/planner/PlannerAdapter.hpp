#ifndef AIKIDO_PLANNER_PLANNERADAPTER_HPP_
#define AIKIDO_PLANNER_PLANNERADAPTER_HPP_

namespace aikido {
namespace planner {

/// Adapts a DelegatePlanner to solve the single problem that TargetPlanner can
/// solve.
/// \tparam DelegatePlanner type of \c SingleProblemPlanner to delegate to
/// \tparam TargetPlanner type of \c SingleProblemPlanner to inherit / implement
template <typename DelegatePlanner, typename TargetPlanner>
class PlannerAdapter : public TargetPlanner
{
public:
  /// Constructor.
  ///
  /// \param[in] planner Delegate planner to use internally
  explicit PlannerAdapter(std::shared_ptr<DelegatePlanner> planner);

  /// Default destructor.
  virtual ~PlannerAdapter() = default;

protected:
  /// Internal planner to delegate planning calls
  std::shared_ptr<DelegatePlanner> mDelegate;
};

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/PlannerAdapter-impl.hpp"

#endif // AIKIDO_PLANNER_PLANNERADAPTER_HPP_
