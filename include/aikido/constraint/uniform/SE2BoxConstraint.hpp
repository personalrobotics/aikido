#ifndef AIKIDO_CONSTRAINT_UNIFORM_SE2BOXCONSTRAINT_HPP_
#define AIKIDO_CONSTRAINT_UNIFORM_SE2BOXCONSTRAINT_HPP_

#include "../../statespace/SE2.hpp"
#include "../Differentiable.hpp"
#include "../Projectable.hpp"
#include "../Sampleable.hpp"
#include "../Testable.hpp"

namespace aikido {
namespace constraint {

/// A BoxConstraint on SE2.
///
/// This class does *not* allow constraint on rotation. For each dimension x and
/// y, this constraint has lowerLimit and upperLimit.
class SE2BoxConstraint : public constraint::Projectable,
                         public constraint::Sampleable,
                         public constraint::Testable
{
public:
  using constraint::Projectable::project;

  /// Constructor.
  /// \param space Space in which this constraint operates.
  /// \param rng Random number generator to be used for sampling.
  /// \param lowerLimits Lower limits on the state, only on x and y.
  /// \param upperLimits Upper limits on the state, only on x and y.
  SE2BoxConstraint(
      std::shared_ptr<statespace::SE2> space,
      std::unique_ptr<common::RNG> rng,
      const Eigen::Vector2d& lowerLimits,
      const Eigen::Vector2d& upperLimits);

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

  // Documentation inherited.
  bool isSatisfied(const statespace::StateSpace::State* state, bool _givenState = false) const override;

  // Documentation inherited.
  bool project(
      const statespace::StateSpace::State* s,
      statespace::StateSpace::State* out) const override;

  // Documentation inherited.
  std::unique_ptr<constraint::SampleGenerator> createSampleGenerator()
      const override;

  /// Returns lower limits of this constraint.
  Eigen::Vector2d getLowerLimits() const;

  /// Returns upper limits of this constraint.
  Eigen::Vector2d getUpperLimits() const;

private:
  std::shared_ptr<statespace::SE2> mSpace;

  std::unique_ptr<common::RNG> mRng;

  /// Lower limits on the state. The first element encodes the rotational limit
  /// and the last two elements encode the translational limits.
  Eigen::Vector3d mLowerLimits;

  /// Upper limits on the state. The first element encodes the rotational limit
  /// and the last two elements encode the translational limits.
  Eigen::Vector3d mUpperLimits;

  // DOFs of joint that have limits, in this case translational DOFs.
  // TODO: Confirm this with Gilwoo
  std::size_t mRnDimension;

  // DOF of the joint
  std::size_t mDimension;
};

} // namespace constraint
} // namespace aikido

#endif // AIKIDO_CONSTRAINT_UNIFORM_SE2BOXCONSTRAINT_HPP_
