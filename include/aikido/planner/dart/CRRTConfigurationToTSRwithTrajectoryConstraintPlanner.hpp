#ifndef AIKIDO_PLANNER_DART_CRRTCONFIGURATIONTOTSRWITHTRAJECTORYCONSTRAINTPLANNER_HPP_
#define AIKIDO_PLANNER_DART_CRRTCONFIGURATIONTOTSRWITHTRAJECTORYCONSTRAINTPLANNER_HPP_

#include "aikido/planner/dart/ConfigurationToTSRPlanner.hpp"
#include "aikido/planner/dart/ConfigurationToTSRwithTrajectoryConstraint.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"
#include "aikido/trajectory/Trajectory.hpp"
#include "aikido/robot/util.hpp"

namespace aikido {
namespace planner {
namespace dart {

class CRRTConfigurationToTSRwithTrajectoryConstraintPlanner
  : public dart::SingleProblemPlanner<
        CRRTConfigurationToTSRwithTrajectoryConstraintPlanner,
        ConfigurationToTSRwithTrajectoryConstraint>
{
public:
  // Expose the implementation of Planner::plan(const Problem&, Result*) in
  // SingleProblemPlanner. Note that plan() of the base class takes Problem
  // while the virtual function defined in this class takes SolvableProblem,
  // which is simply ConfigurationToTSR.
  using SingleProblemPlanner::plan;

  /// Constructor
  ///
  /// \param[in] stateSpace State space that this planner associated with.
  /// \param[in] metaSkeleton MetaSkeleton to use for planning.
  /// \param[in] timelimit Timelimit for planning.
  /// \param[in] crrtParameters CRRT planner parameters.
  /// \param[in] configurationRanker Ranker for goalTSR.
  CRRTConfigurationToTSRwithTrajectoryConstraintPlanner(
      statespace::dart::ConstMetaSkeletonStateSpacePtr stateSpace,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton,
      double timelimit,
      robot::util::CRRTPlannerParameters crrtParameters);

  /// Solves \c problem returning the result to \c result.
  ///
  /// \param[in] problem Planning problem to be solved by the planner.
  /// \param[out] result Result of planning procedure.
  trajectory::TrajectoryPtr plan(
      const SolvableProblem& problem, Result* result = nullptr);
  // Note: SolvableProblem is defined in SingleProblemPlanner.

protected:
  const robot::util::CRRTPlannerParameters mCRRTParameters;

  // Planning timelimit
  const double mTimelimit;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_CRRTCONFIGURATIONTOTSRWITHTRAJECTORYCONSTRAINTPLANNER_HPP_
