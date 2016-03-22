#ifndef AIKIDO_CONSTRAINT_PROJECTABLE_H
#define AIKIDO_CONSTRAINT_PROJECTABLE_H

#include <Eigen/Dense>
#include "../state/State.hpp"

namespace aikido {
namespace constraint{

/// Uses Newton's method to project state
class ProjectableByDifferentiable : public Projectable
{
public:

  ProjectableByDifferentiable(const DifferentiablePtr& _differentiable);

  bool contains(const state::State& _s) const override;

  boost::optional<state::State> project(const state::State& _s) const override;

private:
  DifferentiablePtr mDifferentiable;
};


} // constraint
} // aikido

#endif
