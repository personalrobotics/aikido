#ifndef AIKIDO_PLANNER_DART_PLANNERADAPTER_HPP_
#define AIKIDO_PLANNER_DART_PLANNERADAPTER_HPP_

namespace aikido {
namespace planner {
namespace dart {

/// Adapts a DelegatePlanner to solve the single problem that TargetPlanner can
/// solve.
///
/// \tparam DelegatePlanner type of \c SingleProblemPlanner to delegate to
/// \tparam TargetPlanner type of \c dart::SingleProblemPlanner to implement
template <typename DelegatePlanner,
          typename TargetPlanner,
          typename DelegateIsDartPlanner = void>
class PlannerAdapter : public TargetPlanner
{
public:
  /// Constructor to adapt non-DART planners.
  ///
  /// \param[in] planner Delegate planner to use internally.
  /// \param[in] metaSkeleton MetaSkeleton to use for planning.
  explicit PlannerAdapter(
      std::shared_ptr<DelegatePlanner> planner,
      ::dart::dynamics::MetaSkeletonPtr metaSkeleton);

  /// Default destructor.
  virtual ~PlannerAdapter() = default;

protected:
  /// Internal planner to delegate planning calls
  std::shared_ptr<DelegatePlanner> mDelegate;
};

/// Adapts a DelegatePlanner to solve the single problem that TargetPlanner can
/// solve.
///
/// \tparam DelegatePlanner type of \c dart::SingleProblemPlanner to delegate to
/// \tparam TargetPlanner type of \c dart::SingleProblemPlanner to implement
template <typename DelegatePlanner, typename TargetPlanner>
class
    PlannerAdapter<DelegatePlanner,
                   TargetPlanner,
                   typename std::
                       enable_if<std::
                                     is_base_of<dart::
                                                    SingleProblemPlanner<DelegatePlanner,
                                                                         typename DelegatePlanner::
                                                                             SolvableProblem>,
                                                DelegatePlanner>::value>::type>
{
public:
  /// Constructor to adapt DART planners.
  ///
  /// \param[in] planner DART delegate planner to use internally.
  explicit PlannerAdapter(std::shared_ptr<DelegatePlanner> planner);

  /// Default destructor.
  virtual ~PlannerAdapter() = default;

protected:
  /// Internal planner to delegate planning calls
  std::shared_ptr<DelegatePlanner> mDelegate;
};

} // namespace dart
} // namespace planner
} // namespace aikido

#include "aikido/planner/dart/detail/PlannerAdapter-impl.hpp"

#endif // AIKIDO_PLANNER_DART_PLANNERADAPTER_HPP_
