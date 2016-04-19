
#ifndef PR_CONSTRAINT_OMPL_PLANNERS_GEOMETRIC_PLANNERS_RRT_CRRT_
#define PR_CONSTRAINT_OMPL_PLANNERS_GEOMETRIC_PLANNERS_RRT_CRRT_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>

#include "../constraint/Projectable.hpp"

namespace aikido
{
namespace ompl
{
/// Implements a constraint RRT planner
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
  ::ompl::base::PlannerStatus solve(
      const ::ompl::base::PlannerTerminationCondition &ptc) override;

  /// Clear all internal datastructures. Planner settings are not affected.
  /// Subsequent calls to solve() will ignore all previous work.
  void clear(void) override;

  /// Set the goal bias. In the process of randomly selecting states in the
  /// state space to attempt to go towards, the algorithm may in fact choose the
  /// actual goal state, if it knows it, with some probability. This probability
  /// is a real number between 0.0 and 1.0; its value should usually be around
  /// 0.05 and should not be too large. It is probably a good idea to use the
  /// default value.
  void setGoalBias(double goalBias);

  /// Get the goal bias the planner is using
  double getGoalBias(void) const;

  /// Set the range the planner is supposed to use. This parameter greatly
  /// influences the runtime of the algorithm. It represents the maximum length
  /// of a motion to be added in the tree of motions.
  void setRange(double distance);

  /// Get the range the planner is using
  double getRange(void) const;

  /// Set a projectable constraint to be applied throughout the trajectory
  void setTrajectoryWideConstraint(
      const std::shared_ptr<constraint::Projectable> &_projectable);

  /// Set a nearest neighbors data structure
  template <template <typename T> class NN>
  void setNearestNeighbors(void)
  {
    nn_.reset(new NN<Motion *>());
  }

  /// Perform extra configuration steps, if needed. This call will also issue a
  /// call to ompl::base::SpaceInformation::setup() if needed. This must be
  /// called before solving.
  void setup(void) override;

protected:
  /** \brief Representation of a motion
      This only contains pointers to parent motions as we
      only need to go backwards in the tree. */
  class Motion
  {
  public:
    Motion(void)
        : state(NULL)
        , parent(NULL)
    {
    }

    /** \brief Constructor that allocates memory for the state */
    Motion(const ::ompl::base::SpaceInformationPtr &si)
        : state(si->allocState())
        , parent(NULL)
    {
    }

    ~Motion(void) {}

    /** \brief The state contained by the motion */
    ::ompl::base::State *state;

    /** \brief The parent motion in the exploration tree */
    Motion *parent;
  };

  /** \brief Free the memory allocated by this planner */
  void freeMemory(void);

  /** \brief Compute distance between motions (actually distance between
   * contained states) */
  double distanceFunction(const Motion *a, const Motion *b) const
  {
    return si_->distance(a->state, b->state);
  }

  /** \brief State sampler */
  ::ompl::base::StateSamplerPtr sampler_;

  /** \brief A nearest-neighbors datastructure containing the tree of motions */
  boost::shared_ptr<::ompl::NearestNeighbors<Motion *>> nn_;

  /** \brief The fraction of time the goal is picked as the state to expand
   * towards (if such a state is available) */
  double goalBias_;

  /** \brief The maximum length of a motion to be added to a tree */
  double maxDistance_;

  /** \brief The random number generator */
  ::ompl::RNG rng_;

  /** \brief The most recent goal motion.  Used for PlannerData computation */
  Motion *lastGoalMotion_;

  /** \brief The trajectory-wide PR holonomic constraint to be followed */
  std::shared_ptr<constraint::Projectable> cons_;
};
}
}

#endif
