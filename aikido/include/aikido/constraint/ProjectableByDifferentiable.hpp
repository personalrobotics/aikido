#ifndef AIKIDO_CONSTRAINT_PROJECTABLEBYDIFFERENTIABLE_H
#define AIKIDO_CONSTRAINT_PROJECTABLEBYDIFFERENTIABLE_H

#include <Eigen/Dense>
#include "../state/State.hpp"
#include "Projectable.hpp"
#include "Differentiable.hpp"

namespace aikido {
namespace constraint{

/// Uses Newton's method to project state.
/// Supports only RealVectorState and SE3State.
class ProjectableByDifferentiable : public Projectable
{
public:

  ProjectableByDifferentiable(
  	const std::shared_ptr<const Differentiable>& _differentiable,
  	int _maxIteration=1000);

  bool contains(const state::StatePtr& _s) const override;

  boost::optional<state::StatePtr> project(
  	const state::StatePtr& _s) override;

protected:
  const std::shared_ptr<const Differentiable> mDifferentiable;
  int mMaxIteration;
};


} // constraint
} // aikido

#endif
