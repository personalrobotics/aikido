#ifndef AIKIDO_CONSTRAINT_SATISFIED_HPP_
#define AIKIDO_CONSTRAINT_SATISFIED_HPP_
#include "Differentiable.hpp"
#include "Projectable.hpp"
#include "Sampleable.hpp"
#include "Testable.hpp"

namespace aikido {
namespace constraint {

/// A constraint which is always satisfied.
/// This class is often used in CartesianProduct constraints
/// to represent that some subspace doesn't have any constraint.
class Satisfied
  : public constraint::Differentiable
  , public constraint::Projectable
  , public constraint::Testable
{
public:
  using Differentiable::getValueAndJacobian;

  /// Constructor.
  /// \param _space StateSpace in which this constraint operates.
  explicit Satisfied(statespace::ConstStateSpacePtr _space);

  // Documentation inherited.
  statespace::ConstStateSpacePtr getStateSpace() const override;

  /// Returns \c 0.
  std::size_t getConstraintDimension() const override;

  /// Returns an empty vector.
  std::vector<constraint::ConstraintType> getConstraintTypes() const override;

  /// Returns \c true.
  ///
  /// \param state a state in \c getStateSpace()
  /// \param outcome object to populate with information about satisfiability
  ///        result.
  bool isSatisfied(
      const statespace::StateSpace::State* state,
      TestableOutcome* outcome = nullptr) const override;

  /// Return an instance of DefaultTestableOutcome, since this class doesn't
  /// have a more specialized TestableOutcome derivative assigned to it.
  std::unique_ptr<TestableOutcome> createOutcome() const override;

  /// Sets \c _out to \c _s.
  ///
  /// \param _s input state
  /// \param[out] _out output state
  bool project(
      const statespace::StateSpace::State* _s,
      statespace::StateSpace::State* _out) const override;

  /// Returns an empty vector.
  ///
  /// \param _s input state
  /// \param[out] _out returns an empty vector
  void getValue(const statespace::StateSpace::State* _s, Eigen::VectorXd& _out)
      const override;

  /// Returns an empty matrix.
  ///
  /// \param _s input state
  /// \param[out] _out empty Jacobian matrix
  void getJacobian(
      const statespace::StateSpace::State* _s,
      Eigen::MatrixXd& _out) const override;

private:
  statespace::ConstStateSpacePtr mStateSpace;
};

} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_SATISFIED_HPP_
