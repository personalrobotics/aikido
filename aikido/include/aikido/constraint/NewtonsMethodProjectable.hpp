#ifndef AIKIDO_CONSTRAINT_NEWTONSMETHODPROJECTABLE_HPP_
#define AIKIDO_CONSTRAINT_NEWTONSMETHODPROJECTABLE_HPP_

#include <Eigen/Dense>
#include "Projectable.hpp"
#include "Differentiable.hpp"

namespace aikido {
namespace constraint{

/// Uses Newton's method to project state.
class NewtonsMethodProjectable : public Projectable
{
public:

  /// Constructor.
  /// \param _differentiable Differentiable constraint to be projected. 
  /// \param _tolerance Tolerances for checking whether the constraints
  ///        are been satisfied. e.g. For equality, 
  ///        |_differentiable->getValue(state)| <= tolerance
  ///        The size of tolerances should match _differentiable's constraint
  ///        dimension.
  /// \param _maxIteration Max iteration for Newton's method.
  /// \param _minStepSize Minimum step size to be taken in Newton's method.
  NewtonsMethodProjectable(
    DifferentiablePtr _differentiable,
    std::vector<double> _tolerance,
    int _maxIteration=1000,
    double _minStepSize=1e-5);

  // Documentation inherited.
  bool project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const override;

  // Documentation inherited.
  statespace::StateSpacePtr getStateSpace() const override;

private:
  DifferentiablePtr mDifferentiable;
  statespace::StateSpacePtr mStateSpace;
  int mMaxIteration;
  std::vector<double> mTolerance;
  double mMinStepSize;

  bool contains(const statespace::StateSpace::State* _s) const;
};


} // constraint
} // aikido

#endif
