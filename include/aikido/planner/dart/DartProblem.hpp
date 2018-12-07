#ifndef AIKIDO_PLANNER_DART_DARTPROBLEM_HPP_
#define AIKIDO_PLANNER_DART_DARTPROBLEM_HPP_

#include <dart/dart.hpp>
#include "aikido/planner/Planner.hpp"

namespace aikido {
namespace planner {

AIKIDO_DECLARE_POINTERS(DartProblem)

  /// Base class for various planning problems with DART component.
class DartProblem
{
public:
  /// Clones this planning problem.
  /// \param[in] metaSkeleton MetaSkeleton to be used for cloning
  /// relevant properties.
  virtual std::shared_ptr<Problem> clone
    (::dart::dynamics::MetaSkeletonPtr metaSkeleton) const = 0;
};

} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_DART_DARTPROBLEM_HPP_
