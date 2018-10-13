#include "aikido/planner/Planner.hpp"

#include <utility>

namespace aikido {
namespace planner {

//==============================================================================
Planner::Planner(statespace::ConstStateSpacePtr stateSpace, common::RNG* rng)
  : mStateSpace(std::move(stateSpace))
{
  if (!rng)
    mRng.reset(new common::RNGWrapper<std::mt19937>(std::random_device{}()));
  else
    mRng = std::move(cloneRNGFrom(*rng)[0]);
}

//==============================================================================
statespace::ConstStateSpacePtr Planner::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
common::RNG* Planner::getRng()
{
  return mRng.get();
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
