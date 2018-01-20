#include "aikido/planner/Problem.hpp"

namespace aikido {
namespace planner {

//==============================================================================
Problem::Problem(statespace::StateSpacePtr stateSpace)
  : mStateSpace(std::move(stateSpace))
{
  // Do nothing
}

//==============================================================================
statespace::StateSpacePtr Problem::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
void Problem::Result::setMessage(const std::string& message)
{
  mMessage = message;
}

//==============================================================================
const std::string& Problem::Result::getMessage() const
{
  return mMessage;
}

} // namespace planner
} // namespace aikido
