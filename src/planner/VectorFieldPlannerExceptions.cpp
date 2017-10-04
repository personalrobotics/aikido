#include "VectorFieldPlannerExceptions.h"

namespace aikido {
namespace planner {

/*
 * VectorFieldTerminated
 */
VectorFieldTerminated::VectorFieldTerminated(std::string const &what_arg)
  : std::runtime_error(what_arg)
{
}


/*
 * DofLimitError
 */
DofLimitError::DofLimitError(dart::dynamics::DegreeOfFreedom *dof,
                             std::string const &what_arg)
  : VectorFieldTerminated(what_arg)
  , dof_(dof)
{
}

dart::dynamics::DegreeOfFreedom *DofLimitError::dof() const
{
  return dof_;
}

} // namespace planner
} // namespace aikido
