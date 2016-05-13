#ifndef AIKIDO_PLANNER_OMPL_CRRT_HPP_
#define AIKIDO_PLANNER_OMPL_CRRT_HPP_

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include "../../constraint/Projectable.hpp"

namespace aikido {
namespace planner {
namespace ompl {
/// Implements a constrained RRT planner
class CRRT : public ::ompl::base::Planner
{
public:
  /// Constructor
  CRRT(const ::ompl::base::SpaceInformationPtr &_si);

  /// Destructor
  virtual ~CRRT(void);

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
  ::ompl::base::PlannerStatus solve(
      const ::ompl::base::PlannerTerminationCondition &_ptc) override;

  /// Solve the motion planning problem in the given time
  /// \param solveTime The maximum allowable time to solve the planning problem
  ::ompl::base::PlannerStatus solve(double _solveTime);

  /// Clear all internal datastructures. Planner settings are not affected.
  /// Subsequent calls to solve() will ignore all previous work.
  void clear(void) override;

  /// Set the goal bias. In the process of randomly selecting states in the
  /// state space to attempt to go towards, the algorithm may in fact choose the
  /// actual goal state, if it knows it, with some probability. This probability
  /// is a real number between 0.0 and 1.0; its value should usually be around
  /// 0.05 and should not be too large. It is probably a good idea to use the
  /// default value.
  void setGoalBias(double _goalBias);

  /// Get the goal bias the planner is using
  double getGoalBias(void) const;

  /// Set the range the planner is supposed to use. This parameter greatly
  /// influences the runtime of the algorithm. It represents the maximum length
  /// of a motion to be added in the tree of motions.
  /// \param distance The maximum length of a motion to be added in the tree of
  /// motions
  void setRange(double _distance);

  /// Get the range the planner is using
  double getRange(void) const;

  /// Set a projectable constraint to be applied throughout the trajectory.
  /// The projection is applied after each extension of the tree.  Each extension
  ///  will be at most getRange distance from the node being extend. Therefore,
  ///  the resolution of the projection is the range set on the planner through
  ///  setRange.
  /// \param _projectable The constraint
  void setPathConstraint(
      constraint::ProjectablePtr _projectable);

  /// Set a nearest neighbors data structure
  template <template <typename T> class NN>
  void setNearestNeighbors(void);

  /// Perform extra configuration steps, if needed. This call will also issue a
  /// call to ompl::base::SpaceInformation::setup() if needed. This must be
  /// called before solving.
  void setup(void) override;

protected:
  /// Representation of a node in the tree. Contains the state at the node and a
  /// pointer to the parent node.
  class Motion
  {
  public:
    /// Constructor. Sets state and parent to null ptr.
    Motion(void);

    /// Constructor that allocates memory for the state
    Motion(const ::ompl::base::SpaceInformationPtr &_si);

    /// Destructor
    ~Motion(void);

    /// The state contained in this node
    ::ompl::base::State* state;

    /// The parent of this node
    Motion* parent;
  };

  /// Free the memory allocated by this planner
  void freeMemory(void);

  /// Compute distance between motions (actually distance between contained
  /// states
  double distanceFunction(const Motion *a, const Motion *b) const;

  /// State sampler
  ::ompl::base::StateSamplerPtr mSampler;

  /// A nearest-neighbors datastructure containing the tree of motions
  boost::shared_ptr<::ompl::NearestNeighbors<Motion *>> mNN;

  /// The fraction of time the goal is picked as the state to expand towards (if
  /// such a state is available)
  double mGoalBias;

  /// The maximum length of a motion to be added to a tree
  double mMaxDistance;

  /// The random number generator used to determine whether to sample a goal
  /// state or a state uniformly from free space
  ::ompl::RNG mRng;

  /// The most recent goal motion.  Used for PlannerData computation
  Motion *mLastGoalMotion;

  /// The constraint that must be satisfied throughout the trajectory
  constraint::ProjectablePtr mCons;
};
} // namespace ompl
} // namespace planner
} // namespace aikido

#include "detail/CRRT-impl.hpp"

#endif // AIKIDO_PLANNER_OMPL_CRRT_HPP_
