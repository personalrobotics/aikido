#ifndef AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_
#define AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "../../constraint/Projectable.hpp"

namespace aikido
{
namespace planner
{
namespace ompl
{
/// Implements a bi-direction constrained RRT planner
class CRRTConnect : public ::ompl::base::Planner
{
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
  void getPlannerData(::ompl::base::PlannerData &data) const override;

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
      const ::ompl::base::PlannerTerminationCondition &ptc) override;

  /// Solve the motion planning problem in the given time
  /// \param solveTime The maximum allowable time to solve the planning problem
  ::ompl::base::PlannerStatus solve(double solveTime);

  /// Clear all internal datastructures. Planner settings are not affected.
  /// Subsequent calls to solve() will ignore all previous work.
  void clear(void) override;

  /// Set the range the planner is supposed to use. This parameter greatly
  /// influences the runtime of the algorithm. It represents the maximum length
  /// of a motion to be added in the tree of motions.
  /// \param distance The maximum length of a motionto be added in the tree of
  /// motions
  void setRange(double distance);

  /// Get the range the planner is using
  double getRange(void) const;

  /// Set a projectable constraint to be applied throughout the trajectory
  /// \param _projectable The constraint to apply to the trajectory
  void setTrajectoryWideConstraint(
      constraint::ProjectablePtr _projectable);

  /// Set a nearest neighbors data structure for both the start and goal trees
  template <template <typename T> class NN>
  void setNearestNeighbors(void)
  {
    tStart_.reset(new NN<Motion *>());
    tGoal_.reset(new NN<Motion *>());
  }

  /// Perform extra configuration steps, if needed. This call will also issue a
  /// call to ompl::base::SpaceInformation::setup() if needed. This must be
  /// called before solving.
  void setup(void) override;

protected:
  /** \brief Representation of a motion */
  class Motion
  {
  public:
    Motion(void)
        : root(NULL)
        , state(NULL)
        , parent(NULL)
    {
      parent = NULL;
      state = NULL;
    }

    Motion(const ::ompl::base::SpaceInformationPtr &si)
        : root(NULL)
        , state(si->allocState())
        , parent(NULL)
    {
    }

    ~Motion(void) {}

    const ::ompl::base::State *root;
    ::ompl::base::State *state;
    Motion *parent;
  };

  /** \brief A nearest-neighbor datastructure representing a tree of motions */
  typedef boost::shared_ptr<::ompl::NearestNeighbors<Motion *>> TreeData;

  /** \brief Information attached to growing a tree of motions (used internally)
   */
  struct TreeGrowingInfo {
    ::ompl::base::State *xstate;
    ::ompl::base::State *pstate;
    Motion *xmotion;
    bool start;
  };

  /** \brief The state of the tree after an attempt to extend it */
  enum GrowState {
    /// no progress has been made
    TRAPPED,
    /// progress has been made towards the randomly sampled state
    ADVANCED,
    /// the randomly sampled state was reached
    REACHED
  };

  /** \brief Free the memory allocated by this planner */
  void freeMemory(void);

  /** \brief Compute distance between motions (actually distance between
   * contained states) */
  double distanceFunction(const Motion *a, const Motion *b) const
  {
    return si_->distance(a->state, b->state);
  }

  /** \brief Grow a tree towards a random state */
  GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

  /** \brief State sampler */
  ::ompl::base::StateSamplerPtr sampler_;

  /** \brief The start tree */
  TreeData tStart_;

  /** \brief The goal tree */
  TreeData tGoal_;

  /** \brief The maximum length of a motion to be added to a tree */
  double maxDistance_;

  /** \brief The random number generator */
  ::ompl::RNG rng_;

  /** \brief The pair of states in each tree connected during planning.  Used
   * for PlannerData computation */
  std::pair<::ompl::base::State *, ::ompl::base::State *> connectionPoint_;

  /** \brief The trajectory-wide PR holonomic constraint to be followed */
  constraint::ProjectablePtr cons_;

};
}
}
}
#endif  //AIKIDO_PLANNER_OMPL_CRRTCONNECT_HPP_
