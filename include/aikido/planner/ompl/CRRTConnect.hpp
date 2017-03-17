#ifndef AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_
#define AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_

#include <aikido/planner/ompl/CRRT.hpp>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "../../constraint/Projectable.hpp"

namespace aikido {
namespace planner {
namespace ompl {
/// Implements a bi-direction constrained RRT planner
class CRRTConnect : public CRRT {
public:
  /// Constructor
  /// \param si Information about the planning instance
  CRRTConnect(const ::ompl::base::SpaceInformationPtr &si);

  /// Destructor
  virtual ~CRRTConnect(void);

  /// Get information about the current run of the motion planner. Repeated
  /// calls to this function will update data (only additions are made). This is
  /// useful to see what changed in the exploration datastructure, between calls
  /// to solve(), for example (without calling clear() in between).
  /// \param[out] data Data about the current run of the motion planner
  void getPlannerData(::ompl::base::PlannerData &_data) const override;

  /// Function that can solve the motion planning problem. This function can be
  /// called multiple times on the same problem, without calling clear() in
  /// between. This allows the planner to continue work for more time on an
  /// unsolved problem, for example. If this option is used, it is assumed the
  /// problem definition is not changed (unpredictable results otherwise). The
  /// only change in the problem definition that is accounted for is the
  /// addition of starting or goal states (but not changing previously added
  /// start/goal states). The function terminates if the call to ptc returns
  /// true.
  /// \param ptc Conditions for terminating planning before a solution is found
  ::ompl::base::PlannerStatus
  solve(const ::ompl::base::PlannerTerminationCondition &p_tc) override;

  /// Solve the motion planning problem in the given time
  /// \param solveTime The maximum allowable time to solve the planning problem
  ::ompl::base::PlannerStatus solve(double _solveTime);

  /// Clear all internal datastructures. Planner settings are not affected.
  /// Subsequent calls to solve() will ignore all previous work.
  void clear(void) override;

  /// \param radius The maximum distance between two trees for them to be
  /// considered connected
  void setConnectionRadius(double _radius);

    /// Get the connection radius the planner is using
  double getConnectionRadius(void) const;

  /// Set a nearest neighbors data structure for both the start and goal trees
  template <template <typename T> class NN> void setNearestNeighbors(void);

  /// Perform extra configuration steps, if needed. This call will also issue a
  /// call to ompl::base::SpaceInformation::setup() if needed. This must be
  /// called before solving.
  void setup(void) override;

protected:

  /// Free the memory allocated by this planner
  void freeMemory(void) override;

  /// The goal tree
  TreeData mGoalTree;

  /// Max distance between two trees to consider them connected
  double mConnectionRadius;

  /// The pair of states in each tree connected during planning.  Use for
  /// PlannerData computation
  std::pair<::ompl::base::State *, ::ompl::base::State *> mConnectionPoint;

};
} // namespace ompl
} // namespace planner
} // namespace aikido

#include "detail/CRRTConnect-impl.hpp"

#endif // AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_
