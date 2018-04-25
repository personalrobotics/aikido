#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {

//==============================================================================
Planner::Planner(const statespace::ConstStateSpacePtr& stateSpace)
  : mStateSpace(std::move(stateSpace))
{
  // Do nothing
}

//==============================================================================
statespace::ConstStateSpacePtr Planner::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
void Planner::Result::setMessage(const std::string& message)
{
  mMessage = message;
}

//==============================================================================
const std::string& Planner::Result::getMessage() const
{
  return mMessage;
}

} // namespace planner
} // namespace aikido
