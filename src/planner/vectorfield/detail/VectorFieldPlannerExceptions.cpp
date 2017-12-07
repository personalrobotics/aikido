#include <aikido/planner/vectorfield/detail/VectorFieldPlannerExceptions.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

//==============================================================================
VectorFieldTerminated::VectorFieldTerminated(const std::string& whatArg)
  : mWhatArg(whatArg)
{
  // Do nothing
}

//==============================================================================
const char* VectorFieldTerminated::what() const throw()
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
DofLimitError::DofLimitError() : VectorFieldError("DOF limit exceeded")
{
  // Do nothing
}

//==============================================================================
ConstraintViolatedError::ConstraintViolatedError()
  : VectorFieldError("State in collision")
{
  // Do nothing
}

//==============================================================================
IntegrationFailedError::IntegrationFailedError()
  : VectorFieldError("Integation failed.")
{
  // Do nothing
}

//==============================================================================
TimeLimitError::TimeLimitError() : VectorFieldError("Reached time limit.")
{
  // DO nothing
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
