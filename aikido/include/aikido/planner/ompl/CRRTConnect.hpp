#ifndef AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_
#define AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "../../constraint/Projectable.hpp"

namespace aikido {
namespace planner {
namespace ompl {
/// Implements a bi-direction constrained RRT planner
class CRRTConnect : public ::ompl::base::Planner {
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

  /// Set the range the planner is supposed to use. This parameter greatly
  /// influences the runtime of the algorithm. It represents the maximum length
  /// of a motion to be added in the tree of motions.
  /// \param distance The maximum length of a motionto be added in the tree of
  /// motions
  void setRange(double _distance);

  /// Get the range the planner is using
  double getRange(void) const;

  /// Set a projectable constraint to be applied throughout the trajectory
  /// \param _projectable The constraint to apply to the trajectory
  void setTrajectoryWideConstraint(constraint::ProjectablePtr _projectable);

  /// Set a nearest neighbors data structure for both the start and goal trees
  template <template <typename T> class NN> void setNearestNeighbors(void);

  /// Perform extra configuration steps, if needed. This call will also issue a
  /// call to ompl::base::SpaceInformation::setup() if needed. This must be
  /// called before solving.
  void setup(void) override;

protected:
  /// Represents a node in the tree
  class Motion {
  public:
    /// Constructor. root, paret and state all intiialized to null
    Motion(void);

    /// Constructor. The state node is initialized with a newly allocated state.
    /// \param si The SpaceInformation to use to alloc the state
    Motion(const ::ompl::base::SpaceInformationPtr &si);

    /// Destructor
    ~Motion(void);

    /// The root of the tree this node is a part of
    const ::ompl::base::State *root;

    /// The state represented by this node
    ::ompl::base::State *state;

    /// The parent of this node in the tree
    Motion *parent;
  };

  /// A nearest-neighbor datastructure representing a tree of motions */
  typedef boost::shared_ptr<::ompl::NearestNeighbors<Motion *>> TreeData;

  /// Information attached to growing a tree of motions (used internally)
  struct TreeGrowingInfo {
    ::ompl::base::State *xstate;
    ::ompl::base::State *pstate;
    Motion *xmotion;
    bool start;
  };

  /// The state of the tree after an attempt to extend it */
  enum GrowState {
    /// no progress has been made
    TRAPPED,
    /// progress has been made towards the randomly sampled state
    ADVANCED,
    /// the randomly sampled state was reached
    REACHED
  };

  /// Free the memory allocated by this planner
  void freeMemory(void);

  /// Compute distance between motions (actually distance between contained
  /// states)
  double distanceFunction(const Motion *a, const Motion *b) const;

  /// Grow a tree towards a random state
  GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

  /// State sampler
  ::ompl::base::StateSamplerPtr mSampler;

  /// Start tree
  TreeData mStartTree;

  /// The goal tree
  TreeData mGoalTree;

  /// maximum length of a motion to be added to a tree
  double mMaxDistance;

  /// Max distance between two trees to consider them connected
  double mConnectionRadius;

  /// The random number generator
  ::ompl::RNG mRNG;

  /// The pair of states in each tree connected during planning.  Use for
  /// PlannerData computation
  std::pair<::ompl::base::State *, ::ompl::base::State *> mConnectionPoint;

  /// The trajectory-wide PR holonomic constraint to be followed
  constraint::ProjectablePtr mCons;
};
}
}
}

#include "detail/CRRTConnect-impl.hpp"

#endif // AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_
