#include <aikido/planner/vectorfield/detail/VectorFieldPlannerExceptions.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

VectorFieldTerminated::VectorFieldTerminated(const std::string& _whatArg)
  : std::runtime_error(_whatArg)
{
}

//==============================================================================

DofLimitError::DofLimitError(
    dart::dynamics::DegreeOfFreedom* _dof, const std::string& _whatArg)
  : VectorFieldTerminated(_whatArg), mDof(_dof)
{
}

//==============================================================================

dart::dynamics::DegreeOfFreedom* DofLimitError::dof() const
{
  return mDof;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
