#ifndef AIKIDO_STATESPACE_RNBOXCONSTRAINT_H_
#define AIKIDO_STATESPACE_RNBOXCONSTRAINT_H_
#include "../../statespace/Rn.hpp"
#include "../Differentiable.hpp"
#include "../Projectable.hpp"
#include "../Sampleable.hpp"
#include "../Testable.hpp"

namespace aikido {
namespace constraint {

/// A BoxConstraint on RealVectorStates.
/// For each dimension, this constraint has lowerLimit and upperLimit. 
class RnBoxConstraint
  : public constraint::Differentiable
  , public constraint::Projectable
  , public constraint::Sampleable
  , public constraint::Testable
{
public:
  using constraint::Projectable::project;
  using constraint::Differentiable::getValueAndJacobian;

  /// Constructor.
  /// \param _space Space in which this constraint operates.
  /// \param _rng Random number generator to be used for sampling.
  /// \param _lowerLimits Lower limits on the states.
  ///        The length of this vector should match the dimension of _space.
  /// \param _upperLimits Upper limits.
  ///        The length of this vector should match the dimension of _space. 
  RnBoxConstraint(
    std::shared_ptr<statespace::Rn> _space,
    std::unique_ptr<util::RNG> _rng,
    const Eigen::VectorXd& _lowerLimits,
    const Eigen::VectorXd& _upperLimits);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  size_t getConstraintDimension() const override;

  // Documentation inherited.
  std::vector<constraint::ConstraintType> getConstraintTypes() const override;

  // Documentation inherited.
  bool isSatisfied(const statespace::StateSpace::State* state) const override;

  // Documentation inherited.
  bool project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const override;

  // Documentation inherited.
  void getValue(
    const statespace::StateSpace::State* _s,
    Eigen::VectorXd& _out) const override;

  // Documentation inherited.
  void getJacobian(
    const statespace::StateSpace::State* _s,
    Eigen::MatrixXd& _out) const override;
  
  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator>
    createSampleGenerator() const override;

  /// Returns lower limits of this constraint.
  const Eigen::VectorXd& getLowerLimits() const;

  /// Returns upper limits of this constraint.
  const Eigen::VectorXd& getUpperLimits() const;

private:
  std::shared_ptr<statespace::Rn> mSpace;
  std::unique_ptr<util::RNG> mRng;
  Eigen::VectorXd mLowerLimits;
  Eigen::VectorXd mUpperLimits;
};


} // namespace constraint
} // namespace aikido

#endif // AIKIDO_STATESPACE_REALVECTORSTATESPACESAMPLEABLECONSTRAINT_H_
