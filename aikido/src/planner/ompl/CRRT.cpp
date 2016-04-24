#include <aikido/planner/ompl/CRRT.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>

namespace aikido {
namespace planner {
namespace ompl {
//=============================================================================
CRRT::CRRT(const ::ompl::base::SpaceInformationPtr &si)
    : ::ompl::base::Planner(si, "CRRT")
    , cons_(std::shared_ptr<constraint::Projectable>())
    , goalBias_(0.05)
    , maxDistance_(0.0)
    , lastGoalMotion_(NULL)
{
  auto ss = boost::dynamic_pointer_cast<GeometricStateSpace>(
      si->getStateSpace());
  if (!ss) {
    throw std::invalid_argument(
        "CRRT algorithm requires an GeometricStateSpace");
  }

  specs_.approximateSolutions = true;
  specs_.directed = true;

  Planner::declareParam<double>("range", this, &CRRT::setRange, &CRRT::getRange,
                                "0.:1.:10000.");
  Planner::declareParam<double>("goal_bias", this, &CRRT::setGoalBias,
                                &CRRT::getGoalBias, "0.:.05:1.");
}

//=============================================================================
CRRT::~CRRT(void) { freeMemory(); }

//=============================================================================
void CRRT::getPlannerData(::ompl::base::PlannerData &data) const
{
  ::ompl::base::Planner::getPlannerData(data);

  std::vector<Motion *> motions;
  if (nn_) nn_->list(motions);

  if (lastGoalMotion_)
    data.addGoalVertex(::ompl::base::PlannerDataVertex(lastGoalMotion_->state));

  for (unsigned int i = 0; i < motions.size(); ++i) {
    if (motions[i]->parent == NULL)
      data.addStartVertex(::ompl::base::PlannerDataVertex(motions[i]->state));
    else
      data.addEdge(::ompl::base::PlannerDataVertex(motions[i]->parent->state),
                   ::ompl::base::PlannerDataVertex(motions[i]->state));
  }
}

//=============================================================================
void CRRT::clear(void)
{
  ::ompl::base::Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nn_) nn_->clear();
  lastGoalMotion_ = NULL;
}

//=============================================================================
void CRRT::setGoalBias(double goalBias) 
{
  goalBias_ = goalBias; 
}

//=============================================================================
double CRRT::getGoalBias() const 
{
  return goalBias_; 
}

//=============================================================================
void CRRT::setRange(double distance) 
{
  maxDistance_ = distance;
}

//=============================================================================
double CRRT::getRange() const 
{ 
  return maxDistance_;
}

//=============================================================================
void CRRT::setTrajectoryWideConstraint(
    const constraint::ProjectablePtr &_projectable)
{
  cons_ = std::move(_projectable);
}

//=============================================================================
void CRRT::setup(void)
{
  ::ompl::base::Planner::setup();
  ::ompl::tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!nn_)
    nn_.reset(::ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(
        si_->getStateSpace()));
  nn_->setDistanceFunction(boost::bind(&CRRT::distanceFunction, this, _1, _2));
}

//=============================================================================
void CRRT::freeMemory(void)
{
  if (nn_) {
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (unsigned int i = 0; i < motions.size(); ++i) {
      if (motions[i]->state) si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

//=============================================================================
::ompl::base::PlannerStatus CRRT::solve(
    const ::ompl::base::PlannerTerminationCondition &ptc)
{
  checkValidity();
  ::ompl::base::Goal *goal = pdef_->getGoal().get();
  ::ompl::base::GoalSampleableRegion *goal_s =
      dynamic_cast<::ompl::base::GoalSampleableRegion *>(goal);

  while (const ::ompl::base::State *st = pis_.nextStart()) {
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    nn_->add(motion);
  }

  if (nn_->size() == 0) {
    // OMPL_ERROR("%s: There are no valid initial states!",
    //                      getName().c_str());
    return ::ompl::base::PlannerStatus::INVALID_START;
  }

  if (!sampler_) sampler_ = si_->allocStateSampler();

  // OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

  Motion *solution = NULL;
  Motion *approxsol = NULL;
  double approxdif = std::numeric_limits<double>::infinity();
  Motion *rmotion = new Motion(si_);
  ::ompl::base::State *rstate = rmotion->state;
  ::ompl::base::State *xstate = si_->allocState();
  ::ompl::base::State *pstate = si_->allocState(); /* projected state */

  while (ptc == false) {
     /* sample random state (with goal biasing) */
    if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
      goal_s->sampleGoal(rstate);
    else
      sampler_->sampleUniform(rstate);

    /* find closest state in the tree */
    Motion *nmotion = nn_->nearest(rmotion);
    ::ompl::base::State *dstate = rstate;

    /* find state to add (initially unconstrained) */
    double d = si_->distance(nmotion->state, rstate);
    if (d > maxDistance_) {
      si_->getStateSpace()->interpolate(nmotion->state, rstate,
                                        maxDistance_ / d, xstate);
      dstate = xstate;
    }

    /* do single-step constraint projection */
    if (cons_) {
      auto dst = dstate->as<GeometricStateSpace::StateType>();
      auto pst = pstate->as<GeometricStateSpace::StateType>();
      cons_->project(dst->mState, pst->mState);
      dstate = pstate;
    }

    if (si_->checkMotion(nmotion->state, dstate)) {
      /* create a motion */
      Motion *motion = new Motion(si_);
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;

      nn_->add(motion);
      double dist = 0.0;
      bool sat = goal->isSatisfied(motion->state, &dist);
      if (sat) {
        approxdif = dist;
        solution = motion;
        break;
      }
      if (dist < approxdif) {
        approxdif = dist;
        approxsol = motion;
      }
    }
  }

  bool solved = false;
  bool approximate = false;
  if (solution == NULL) {
    solution = approxsol;
    approximate = true;
  }

  if (solution != NULL) {
    lastGoalMotion_ = solution;

    /* construct the solution path */
    std::vector<Motion *> mpath;
    while (solution != NULL) {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    /* set the solution path */
    ::ompl::geometric::PathGeometric *path =
        new ::ompl::geometric::PathGeometric(si_);
    for (int i = mpath.size() - 1; i >= 0; --i) path->append(mpath[i]->state);
    pdef_->addSolutionPath(::ompl::base::PathPtr(path), approximate, approxdif);
    solved = true;
  }

  si_->freeState(xstate);
  si_->freeState(pstate);
  if (rmotion->state) si_->freeState(rmotion->state);
  delete rmotion;

  // OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
  return ::ompl::base::PlannerStatus(solved, approximate);
}

//=============================================================================
::ompl::base::PlannerStatus CRRT::solve(double solveTime) {
    return solve(::ompl::base::timedPlannerTerminationCondition(solveTime));//, std::min(solveTime/100., 0.1)));
}
}
}
}
