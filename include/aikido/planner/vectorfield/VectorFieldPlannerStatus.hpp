#ifndef AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNERSTATUS_HPP_
#define AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNERSTATUS_HPP_

namespace aikido {
namespace planner {
namespace vectorfield {

enum class VectorFieldPlannerStatus
{
  TERMINATE,
  CACHE_AND_TERMINATE,
  CACHE_AND_CONTINUE,
  CONTINUE
};

} // namespace vectorfield
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_VECTORFIELD_VECTORFIELDPLANNERSTATUS_HPP_
