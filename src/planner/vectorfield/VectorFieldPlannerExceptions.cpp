#include <aikido/planner/vectorfield/VectorFieldPlannerExceptions.hpp>

namespace aikido {
namespace planner {
namespace vectorfield {

VectorFieldTerminated::VectorFieldTerminated(const std::string& _whatArg)
  : std::runtime_error(_whatArg)
{
  // Do nothing
}

//==============================================================================
DofLimitError::DofLimitError(
    const dart::dynamics::DegreeOfFreedom* _dof, const std::string& _whatArg)
  : VectorFieldTerminated(_whatArg), mDof(_dof)
{
  // Do nothing
}

//==============================================================================
const dart::dynamics::DegreeOfFreedom* DofLimitError::dof() const
{
  return mDof;
}

} // namespace vectorfield
} // namespace planner
} // namespace aikido
