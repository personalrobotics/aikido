#include "aikido/planner/Planner.hpp"

#include <utility>

namespace aikido {
namespace planner {

//==============================================================================
Planner::Planner(
    statespace::ConstStateSpacePtr stateSpace, std::unique_ptr<common::RNG> rng)
  : mStateSpace(std::move(stateSpace)), mRng(std::move(rng))
{
  // Do nothing
}

//==============================================================================
statespace::ConstStateSpacePtr Planner::getStateSpace() const
{
  return mStateSpace;
}

//==============================================================================
std::unique_ptr<common::RNG> Planner::getRng() const
{
  if (!mRng)
    return nullptr;

  // NOTE: We usually use unique pointers to pass RNGs, so cloning is needed
  // here.
  return std::move(cloneRNGFrom(*mRng)[0]);
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
