#ifndef AIKIDO_PLANNER_DART_DARTPLANNERADAPTER_HPP_
#define AIKIDO_PLANNER_DART_DARTPLANNERADAPTER_HPP_

#include "aikido/planner/dart/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"
#include "aikido/planner/dart/PlannerAdapter.hpp"

namespace aikido {
namespace planner {
namespace dart {

/// Adapts a non-DART DelegatePlanner to solve the single problem that
/// TargetPlanner can solve.
//
/// \tparam DelegatePlanner non-DART \c SingleProblemPlanner to delegate to
/// \tparam TargetPlanner DART \c SingleProblemPlanner to implement
template <typename DelegatePlanner, typename TargetPlanner>
class DartPlannerAdapter : public TargetPlanner
{
public:
  /// Constructor.
  ///
  /// \param[in] planner Delegate planner to use internally.
  /// \param[in] metaSkeleton MetaSkeleton to use for planning.
  DartPlannerAdapter(
      std::shared_ptr<DelegatePlanner> planner,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton);

  /// Default destructor.
  virtual ~DartPlannerAdapter() = default;

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  trajectory::TrajectoryPtr plan(
      const typename DelegatePlanner::SolvableProblem& problem,
      Planner::Result* result) override;

protected:
  /// Internal planner to delegate planning calls
  std::shared_ptr<DelegatePlanner> mDelegate;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#include "aikido/planner/dart/detail/DartPlannerAdapter-impl.hpp"

#endif // AIKIDO_PLANNER_DART_DARTPLANNERADAPTER_HPP_
