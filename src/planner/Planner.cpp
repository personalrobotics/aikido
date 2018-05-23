#include "aikido/planner/Planner.hpp"

#include <utility>

namespace aikido {
namespace planner {

//==============================================================================
Planner::Planner(statespace::ConstStateSpacePtr stateSpace)
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
Planner::Result::Result(const std::string& message) : mMessage(message)
{
  // Do nothing
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
