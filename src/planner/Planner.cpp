#include "aikido/planner/Planner.hpp"

#include <utility>

namespace aikido {
namespace planner {

//==============================================================================
Planner::Planner(statespace::ConstStateSpacePtr stateSpace, common::RNG* rng)
  : mStateSpace(std::move(stateSpace)), mRng(std::move(rng))
{
  if (!mRng)
  {
    auto defaultRng
        = aikido::common::RNGWrapper<std::mt19937>(std::random_device{}());
    mRng = &defaultRng;
  }
}

//==============================================================================
statespace::ConstStateSpacePtr Planner::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
common::RNG* Planner::getRng() const
{
  return mRng;
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
