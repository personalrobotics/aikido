#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

VectorFieldTerminated::VectorFieldTerminated(const std::string& _whatArg)
  : mWhatArg(_whatArg)
{
  // Do nothing
}

//==============================================================================
const char* VectorFieldTerminated::what() const throw()
{
  return mWhatArg.c_str();
}

//==============================================================================
VectorFieldError::VectorFieldError(const std::string& _whatArg)
  : std::runtime_error(_whatArg)
{
  // Do nothing
}

//==============================================================================
DofLimitError::DofLimitError(
    const dart::dynamics::DegreeOfFreedom* _dof, const std::string& _whatArg)
  : VectorFieldError(_whatArg), mDof(_dof)
{
  // Do nothing
}

//==============================================================================
const dart::dynamics::DegreeOfFreedom* DofLimitError::dof() const
{
  return mDof;
}

//==============================================================================
StateInCollisionError::StateInCollisionError()
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
