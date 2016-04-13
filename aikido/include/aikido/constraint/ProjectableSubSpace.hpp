#ifndef AIKIDO_CONSTRAINT_PROJECTABLESUBSPACE_H_
#define AIKIDO_CONSTRAINT_PROJECTABLESUBSPACE_H_
#include "Projectable.hpp"
#include "../statespace/CompoundStateSpace.hpp"

namespace aikido {
namespace constraint {

class ProjectableSubSpace : public Projectable
{
public:
  ProjectableSubSpace(
    std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
    std::vector<ProjectablePtr> _constraints);

  statespace::StateSpacePtr getStateSpace() const override;

  bool project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const override;

private:
  std::shared_ptr<statespace::CompoundStateSpace> mStateSpace;
  std::vector<ProjectablePtr> mConstraints;
};


} // namespace constraint
} // namespace aikido

#endif // ifndef AIKIDO_CONSTRAINT_PROJECTABLESUBSPACE_H_
