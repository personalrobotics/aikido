#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELD_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELD_HPP_

#include <aikido/constraint/Testable.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlannerStatus.hpp>
#include <aikido/statespace/StateSpace.hpp>

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
  VectorField(const aikido::statespace::StateSpacePtr stateSpace);

  /// Vectorfield callback function.
  ///
  /// \param[in] state Statespace state.
  /// \param[out] qd Joint velocities.
  /// \return Whether joint velocities are successfully computed.
  virtual bool evaluateVelocity(
      const aikido::statespace::StateSpace::State* state,
      Eigen::VectorXd& qd) const = 0;

  /// Vectorfield planning status callback function
  ///
  /// \return Status of planning.
  virtual VectorFieldPlannerStatus evaluateStatus(
      const aikido::statespace::StateSpace::State* state) const = 0;

  /// Get testable for state space limits.
  ///
  /// \return Testable for state space limits.
  virtual aikido::constraint::TestablePtr getBoundTestable() const = 0;

  /// Returns meta skeleton.
  aikido::statespace::StateSpacePtr getStateSpace();

  /// Returns const meta skeleton.
  aikido::statespace::ConstStateSpacePtr getStateSpace() const;

protected:
  /// State space
  aikido::statespace::StateSpacePtr mStateSpace;
};

using VectorFieldPtr = std::shared_ptr<VectorField>;

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELD_HPP_
