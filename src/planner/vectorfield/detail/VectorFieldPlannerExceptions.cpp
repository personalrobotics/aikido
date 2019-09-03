#include "VectorFieldPlannerExceptions.hpp"

namespace aikido {
namespace planner {
namespace vectorfield {
namespace detail {

//==============================================================================
VectorFieldTerminated::VectorFieldTerminated(const std::string& whatArg)
  : mWhatArg(whatArg)
{
  // Do nothing
}

//==============================================================================
const char* VectorFieldTerminated::what() const noexcept
{
  return mWhatArg.c_str();
}

//==============================================================================
VectorFieldError::VectorFieldError(const std::string& whatArg)
  : std::runtime_error(whatArg)
{
  // Do nothing
}

//==============================================================================
ConstraintViolatedError::ConstraintViolatedError()
  : VectorFieldTerminated("Constraint violated.")
{
  // Do nothing
}

//==============================================================================
IntegrationFailedError::IntegrationFailedError()
  : VectorFieldTerminated("Integation failed.")
{
  // Do nothing
}

//==============================================================================
TimeLimitError::TimeLimitError() : VectorFieldTerminated("Reached time limit.")
{
  // DO nothing
}

} // namespace detail
} // namespace vectorfield
} // namespace planner
} // namespace aikido
