#include <sstream>
#include <aikido/constraint/ProjectableSubSpace.hpp>

namespace aikido {
namespace constraint {

ProjectableSubSpace::ProjectableSubSpace(
      std::shared_ptr<statespace::CompoundStateSpace> _stateSpace,
      std::vector<ProjectablePtr> _constraints)
  : mStateSpace(std::move(_stateSpace))
  , mConstraints(std::move(_constraints))
{
  if (!mStateSpace)
    throw std::invalid_argument("CompoundStateSpace is nullptr.");

  if (mConstraints.size() != mStateSpace->getNumStates())
  {
    std::stringstream msg;
    msg << "Number of constraints does not match the number of subspaces:"
        << " expected " << mStateSpace->getNumStates() << ", got "
        << mConstraints.size();
    throw std::invalid_argument(msg.str());
  }

  for (size_t i = 0; i < mConstraints.size(); ++i)
  {
    if (!mConstraints[i])
    {
      std::stringstream msg;
      msg << "Constraint " << i << " is nullptr.";
      throw std::invalid_argument(msg.str());
    }

    if (mConstraints[i]->getStateSpace() != mStateSpace->getSubSpace<>(i))
    {
      std::stringstream msg;
      msg << "Constraint " << i << " operates on the wrong state space.";
      throw std::invalid_argument(msg.str());
    }
  }
}

statespace::StateSpacePtr ProjectableSubSpace::getStateSpace() const
{
  return mStateSpace;
}

bool ProjectableSubSpace::project(const statespace::StateSpace::State* _s,
  statespace::StateSpace::State* _out) const
{
  auto out = static_cast<statespace::CompoundStateSpace::State*>(_out);

  for (size_t i = 0; i < mConstraints.size(); ++i)
  {
    auto outSubState = mStateSpace->getSubState<>(out, i);
    if (!mConstraints[i]->project(_s, outSubState))
      return false;
  }

  return true;
}

} // namespace constraint
} // namespace aikido
