#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNERSTATUS_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNERSTATUS_HPP_

namespace aikido {
namespace planner {
namespace vectorfield {

/// Status of planning
/// TERMINATE - stop gracefully and output the CACHEd trajectory
/// CACHE_AND_CONTINUE - save the current trajectory and CONTINUE.
///                      return the saved trajectory if TERMINATEd.
/// CONTINUE - keep going
/// CACHE_AND_TERMINATE - save the current trajectory and TERMINATE
enum class VectorFieldPlannerStatus
{
  TERMINATE,
  CACHE_AND_CONTINUE,
  CONTINUE,
  CACHE_AND_TERMINATE
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNERSTATUS_HPP_
