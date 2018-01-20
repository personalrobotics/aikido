#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELD_HPP_

#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/trajectory/Trajectory.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

/// This class defines a vector field.
///
/// Any vector field should inherit this class to implememnt functions
/// (1) evaluateVelocity() that calculates velocity given a state; and
/// (2) evaluteStatus() that checks the planner status given a state.
class VectorField
{
public:
  /// Constructor.
  ///
  /// \param[in] stateSpace State space that vector field is defined in.
  explicit VectorField(aikido::statespace::StateSpacePtr stateSpace);

  /// Vectorfield callback function.
  ///
  /// \param[in] state Statespace state.
  /// \param[out] qd Joint velocities.
  /// \return Whether joint velocities are successfully computed.
  virtual bool evaluateVelocity(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& qd) const = 0;

  /// Vectorfield planning status callback function.
  ///
  /// \praram[in] state State to evaluate.
  /// \return Status of planning.
  virtual VectorFieldPlannerStatus evaluateStatus(
      const aikido::statespace::StateSpace::State* state) const = 0;

  /// Evaludate whether a trajectory satisfies a constraint.
  /// It is checked by a user-defined evaluation step size.
  ///
  /// \param[in] trajectory Trajectory to be evaluated.
  /// \param[in] constraint Constraint to be satisfied.
  /// \param[in] evalStepSize The step size used in evaluating constraint.
  /// \param[in/out] evalTimePivot Input provides the start time of the
  /// trajectory
  /// to evaluate; output returns the end time of the trajectory evaluate.
  /// \param[in] excludeEndTime Whether end time is excluded in evaluation.
  /// evaluate.
  /// satisfaction.
  virtual bool evaluateTrajectory(
      const aikido::trajectory::Trajectory& trajectory,
      const aikido::constraint::Testable* constraint,
      double evalStepSize,
      double& evalTimePivot,
      bool excludeEndTime) const = 0;

  /// Returns state space.
  aikido::statespace::StateSpacePtr getStateSpace();

  /// Returns const state space.
  aikido::statespace::ConstStateSpacePtr getStateSpace() const;

protected:
  /// State space
  aikido::statespace::StateSpacePtr mStateSpace;
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELD_HPP_
