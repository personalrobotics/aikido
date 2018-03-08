#include "aikido/planner/PlanningResult.hpp"

#include <cassert>

namespace aikido {
namespace planner {

//==============================================================================
void PlanningResult::setMessage(const std::string& message)
{
  mMessage = message;
}

//==============================================================================
const std::string& PlanningResult::getMessage() const
{
  return mMessage;
}

} // namespace planner
} // namespace aikido
