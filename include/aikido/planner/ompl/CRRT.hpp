#ifndef AIKIDO_PLANNER_OMPL_CRRT_HPP_
#define AIKIDO_PLANNER_OMPL_CRRT_HPP_

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include "../../planner/ompl/BackwardCompatibility.hpp"
#include "../../constraint/Projectable.hpp"

namespace aikido {
namespace planner {
namespace ompl {
/// Implements a constrained RRT planner
class CRRT : public ::ompl::base::Planner
{
public:
  /// Constructor
  /// \param _si Information about the planning space
  CRRT(const ::ompl::base::SpaceInformationPtr &_si);

  /// Constructor
  /// \param _si Information about the planning space
  /// \param _name A name for this planner
  CRRT(const ::ompl::base::SpaceInformationPtr &_si, const std::string &name);

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
  /// The projection is applied at the resolution set via
  /// setProjectionResolution
  /// \param _projectable The constraint
  void setPathConstraint(
      constraint::ProjectablePtr _projectable);

  /// Set the resolution for the projection. During tree extension, a projection
  /// back to the constraint will be performed after any step larger than this
  /// distance.
  /// \param _resolution The max step on an extension before projecting back to
  /// constraint
  void setProjectionResolution(double _resolution);

  /// Get the resolution for the projection. During tree extension, a
  /// projection back to the constraint will be performed after any step
  /// larger than this distance.
  double getProjectionResolution(void) const;

  /// Set the minimum distance between two states for them to be considered
  /// "equivalent". This is used during extension to determine if a projection
  /// is near enough the previous projection to say progress is no longer being
  /// made and quit extending.
  void setMinStateDifference(double _mindist);

  /// Get the minimum distance between two states for them to be considered
  /// "equivalent". This is used during extension to determine if a projection
  /// is near enough the previous projection to say progress is no longer being
  /// made and quit extending.
  double getMinStateDifference(void) const;

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
    Motion(void) : state(nullptr), parent(nullptr) {}

    /// Constructor that allocates memory for the state
    Motion(const ::ompl::base::SpaceInformationPtr &_si)
        : state(_si->allocState()), parent(nullptr) {}

    /// Destructor
    ~Motion(void) {}

    /// The state contained in this node
    ::ompl::base::State* state;

    /// The parent of this node
    Motion* parent;
  };

  /// Free the memory allocated by this planner
  virtual void freeMemory(void);

  /// Compute distance between motions (actually distance between contained
  /// states
  double distanceFunction(const Motion *a, const Motion *b) const;


  /// A nearest-neighbor datastructure representing a tree of motions */
  using TreeData = ompl_shared_ptr<::ompl::NearestNeighbors<Motion *>>;

  /// A nearest-neighbors datastructure containing the tree of motions
  TreeData mStartTree;

  /// Perform an extension that projects to a constraint
  /// \param ptc Planner termination conditions. Used to stop extending if
  /// planning time expires.
  /// \param tree The tree to extend
  /// \param nmotion The node in the tree to extend from
  /// \param gstate The state the extension aims to reach
  /// \param xstate A temporary state that can be used during extension
  /// \param goal The goal of the planning instance
  /// \param returnlast If true, return the last node added to the tree,
  /// otherwise return the node added that was nearest the goal
  /// \param[out] dist The closest distance this extension got to the goal
  /// \param[out] True if the extension reached the goal.
  /// \return fmotion If returnlast is true, the last node on the extension,
  /// otherwise the closest node along the extension to the goal
  Motion *constrainedExtend(
      const ::ompl::base::PlannerTerminationCondition &ptc, TreeData &tree,
      Motion *nmotion, ::ompl::base::State *gstate, ::ompl::base::State *xstate,
      ::ompl::base::Goal *goal, bool returnlast, double &dist, bool &foundgoal);

  /// State sampler
  ::ompl::base::StateSamplerPtr mSampler;

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

  /// The maximum length of a step before projecting
  double mMaxStepsize;

  /// The minumum step size along the constraint. Used to determine
  /// when projection is no longer making progress during an extension.
  double mMinStepsize;
};
} // namespace ompl
} // namespace planner
} // namespace aikido

#include "detail/CRRT-impl.hpp"

#endif // AIKIDO_PLANNER_OMPL_CRRT_HPP_
