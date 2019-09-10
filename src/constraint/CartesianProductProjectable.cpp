#include "aikido/constraint/CartesianProductProjectable.hpp"

#include <sstream>

namespace aikido {
namespace constraint {

//==============================================================================
CartesianProductProjectable::CartesianProductProjectable(
    std::shared_ptr<const statespace::CartesianProduct> _stateSpace,
    std::vector<ProjectablePtr> _constraints)
  : mStateSpace(std::move(_stateSpace)), mConstraints(std::move(_constraints))
{
  if (!mStateSpace)
    throw std::invalid_argument("CartesianProduct is nullptr.");

  if (mConstraints.size() != mStateSpace->getNumSubspaces())
  {
    std::stringstream msg;
    msg << "Number of constraints does not match the number of subspaces:"
        << " expected " << mStateSpace->getNumSubspaces() << ", got "
        << mConstraints.size();
    throw std::invalid_argument(msg.str());
  }

  for (std::size_t i = 0; i < mConstraints.size(); ++i)
  {
    if (!mConstraints[i])
    {
      std::stringstream msg;
      msg << "Constraint " << i << " is nullptr.";
      throw std::invalid_argument(msg.str());
    }

    if (mConstraints[i]->getStateSpace() != mStateSpace->getSubspace<>(i))
    {
      std::stringstream msg;
      msg << "Constraint " << i << " operates on the wrong state space.";
      throw std::invalid_argument(msg.str());
    }
  }
}

//==============================================================================
statespace::ConstStateSpacePtr CartesianProductProjectable::getStateSpace()
    const
{
  return mStateSpace;
}

//==============================================================================
bool CartesianProductProjectable::project(
    const statespace::StateSpace::State* _s,
    statespace::StateSpace::State* _out) const
{
  auto s = static_cast<const statespace::CartesianProduct::State*>(_s);
  auto out = static_cast<statespace::CartesianProduct::State*>(_out);

  for (std::size_t i = 0; i < mConstraints.size(); ++i)
  {
    auto inSubState = mStateSpace->getSubState<>(s, i);
    auto outSubState = mStateSpace->getSubState<>(out, i);

    if (!mConstraints[i]->project(inSubState, outSubState))
      return false;
  }

  return true;
}

} // namespace constraint
} // namespace aikido
