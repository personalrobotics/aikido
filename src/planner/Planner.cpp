#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {

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
