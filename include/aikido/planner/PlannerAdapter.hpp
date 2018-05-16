#ifndef AIKIDO_PLANNER_PLANNERADAPTER_HPP_
#define AIKIDO_PLANNER_PLANNERADAPTER_HPP_

#include "aikido/planner/ConfigurationToConfiguration.hpp"
#include "aikido/planner/ConfigurationToConfigurationPlanner.hpp"
#include "aikido/planner/ConfigurationToConfigurations.hpp"
#include "aikido/planner/SingleProblemPlanner.hpp"
// #include "aikido/planner/ConfigurationToConfigurationsPlanner.hpp"
#include "aikido/planner/ConfigurationToEndEffectorOffset.hpp"
// #include "aikido/planner/ConfigurationToEndEffectorOffsetPlanner.hpp"
#include "aikido/planner/ConfigurationToEndEffectorPose.hpp"
// #include "aikido/planner/ConfigurationToEndEffectorPosePlanner.hpp"
#include "aikido/planner/ConfigurationToTSR.hpp"
#include "aikido/planner/ConfigurationToTSRPlanner.hpp"

namespace aikido {
namespace planner {

template <typename DelegatePlanner>
class PlannerAdapter : public Planner
{
public:
  /// Constructor.
  ///
  /// \param[in] planner Delegate planner to use internally
  explicit PlannerAdapter(std::shared_ptr<DelegatePlanner> planner);

  /// Default destructor.
  virtual ~PlannerAdapter() = default;

  /// Returns true if this planner can solve \c problem.
  virtual bool canSolve(const Problem& problem) const override;

  /// Solves a ConfigurationToConfiguration \c problem using the
  /// DelegatePlanner.
  ///
  /// \param[in] problem Planning problem to solve.
  /// \param[out] result Result of planning.
  /// \return Trajectory or \c nullptr if planning failed.
  virtual trajectory::TrajectoryPtr plan(
      const ConfigurationToConfiguration& problem,
      Planner::Result* result = nullptr);

  /// Solves a ConfigurationToConfigurations \c problem using the
  /// DelegatePlanner.
  ///
  /// \param[in] problem Planning problem to solve.
  /// \param[out] result Result of planning.
  /// \return Trajectory or \c nullptr if planning failed.
  virtual trajectory::TrajectoryPtr plan(
      const ConfigurationToConfigurations& problem,
      Planner::Result* result = nullptr);

  /// Solves a ConfigurationToEndEffectorOffset \c problem using the
  /// DelegatePlanner.
  ///
  /// \param[in] problem Planning problem to solve.
  /// \param[out] result Result of planning.
  /// \return Trajectory or \c nullptr if planning failed.
  virtual trajectory::TrajectoryPtr plan(
      const ConfigurationToEndEffectorOffset& problem,
      Planner::Result* result = nullptr);

  /// Solves a ConfigurationToEndEffectorPose \c problem using the
  /// DelegatePlanner.
  ///
  /// \param[in] problem Planning problem to solve.
  /// \param[out] result Result of planning.
  /// \return Trajectory or \c nullptr if planning failed.
  virtual trajectory::TrajectoryPtr plan(
      const ConfigurationToEndEffectorPose& problem,
      Planner::Result* result = nullptr);

  /// Solves ConfigurationToTSR \c problem using the DelegatePlanner.
  ///
  /// \param[in] problem Planning problem to solve.
  /// \param[out] result Result of planning.
  /// \return Trajectory or \c nullptr if planning failed.
  trajectory::TrajectoryPtr plan(
      const ConfigurationToTSR& problem, Planner::Result* result = nullptr);

  /// Solves \c problem using the DelegatePlanner.
  ///
  /// \param[in] problem Planning problem to solve.
  /// \param[out] result Result of planning.
  /// \return Trajectory or \c nullptr if planning failed.
  virtual trajectory::TrajectoryPtr plan(
      const Problem& problem, Planner::Result* result = nullptr) override;

protected:
  /// Internal planner to delegate planning calls
  std::shared_ptr<DelegatePlanner> mDelegate;
};

using ConfigurationToConfigurationAdapter
    = PlannerAdapter<ConfigurationToConfigurationPlanner>;

} // namespace planner
} // namespace aikido

#include "aikido/planner/detail/PlannerAdapter-impl.hpp"

#include "aikido/planner/detail/ConfigurationToConfigurationAdapter-impl.hpp"

#endif // AIKIDO_PLANNER_PLANNERADAPTER_HPP_
