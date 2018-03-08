#include "aikido/planner/Problem.hpp"

namespace aikido {
namespace planner {

//==============================================================================
Problem::Problem(statespace::ConstStateSpacePtr stateSpace)
  : mStateSpace(std::move(stateSpace))
{
  // Do nothing
}

//==============================================================================
statespace::ConstStateSpacePtr Problem::getStateSpace() const
{
  return mStateSpace;
}

} // namespace planner
} // namespace aikido
