#include <aikido/planner/ompl/CRRT.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <limits>

namespace aikido {
namespace planner {
namespace ompl {
//=============================================================================
CRRT::CRRT(const ::ompl::base::SpaceInformationPtr &_si)
    : ::ompl::base::Planner(_si, "CRRT"), mCons(nullptr), mGoalBias(0.05),
      mMaxDistance(0.0), mLastGoalMotion(nullptr) {
  auto ss =
      boost::dynamic_pointer_cast<GeometricStateSpace>(si_->getStateSpace());
  if (!ss) {
    throw std::invalid_argument(
        "CRRT algorithm requires a GeometricStateSpace");
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
void CRRT::getPlannerData(::ompl::base::PlannerData &_data) const {
  ::ompl::base::Planner::getPlannerData(_data);

  std::vector<Motion *> motions;
  if (mNN)
    mNN->list(motions);

  if (mLastGoalMotion)
    _data.addGoalVertex(
        ::ompl::base::PlannerDataVertex(mLastGoalMotion->state));

  for (unsigned int i = 0; i < motions.size(); ++i) {
    if (motions[i]->parent == nullptr)
      _data.addStartVertex(::ompl::base::PlannerDataVertex(motions[i]->state));
    else
      _data.addEdge(::ompl::base::PlannerDataVertex(motions[i]->parent->state),
                    ::ompl::base::PlannerDataVertex(motions[i]->state));
  }
}

//=============================================================================
void CRRT::clear(void) {
  ::ompl::base::Planner::clear();
  mSampler.reset();
  freeMemory();
  if (mNN)
    mNN->clear();
  mLastGoalMotion = nullptr;
}

//=============================================================================
void CRRT::setGoalBias(double _goalBias)
{ 
  mGoalBias = _goalBias;
}

//=============================================================================
double CRRT::getGoalBias() const
{ 
  return mGoalBias;
}

//=============================================================================
void CRRT::setRange(double _distance){
  mMaxDistance = _distance;
}

//=============================================================================
double CRRT::getRange() const
{
  return mMaxDistance;
}

//=============================================================================
void CRRT::setTrajectoryWideConstraint(
    constraint::ProjectablePtr _projectable) {
  mCons = std::move(_projectable);
}

//=============================================================================
void CRRT::setup(void) {
  ::ompl::base::Planner::setup();
  ::ompl::tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(mMaxDistance);

  if (!mNN)
    mNN.reset(::ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(
        si_->getStateSpace()));
  mNN->setDistanceFunction(boost::bind(&CRRT::distanceFunction, this, _1, _2));
}

//=============================================================================
void CRRT::freeMemory(void) {
  if (mNN) {
    std::vector<Motion *> motions;
    mNN->list(motions);
    for (unsigned int i = 0; i < motions.size(); ++i) {
      if (motions[i]->state)
        si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

//=============================================================================
::ompl::base::PlannerStatus
CRRT::solve(const ::ompl::base::PlannerTerminationCondition &_ptc) {
  checkValidity();
  ::ompl::base::Goal *goal = pdef_->getGoal().get();
  ::ompl::base::GoalSampleableRegion *goalSampleable =
      dynamic_cast<::ompl::base::GoalSampleableRegion *>(goal);

  while (const ::ompl::base::State *st = pis_.nextStart()) {
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    mNN->add(motion);
  }

  if (mNN->size() == 0) {
    return ::ompl::base::PlannerStatus::INVALID_START;
  }

  if (!mSampler)
    mSampler = si_->allocStateSampler();

  Motion *solution = nullptr;
  Motion *approxsol = nullptr;
  double approxdif = std::numeric_limits<double>::infinity();
  Motion *rmotion = new Motion(si_);
  ::ompl::base::State *rstate = rmotion->state;
  ::ompl::base::State *xstate = si_->allocState();
  ::ompl::base::State *pstate = si_->allocState(); /* projected state */

  while (_ptc == false) {
    /* sample random state (with goal biasing) */
    if (goalSampleable && mRng.uniform01() < mGoalBias && goalSampleable->canSample())
      goalSampleable->sampleGoal(rstate);
    else
      mSampler->sampleUniform(rstate);

    /* find closest state in the tree */
    Motion *nmotion = mNN->nearest(rmotion);
    ::ompl::base::State *dstate = rstate;

    /* find state to add (initially unconstrained) */
    double d = si_->distance(nmotion->state, rstate);
    if (d > mMaxDistance) {
      si_->getStateSpace()->interpolate(nmotion->state, rstate,
                                        mMaxDistance / d, xstate);
      dstate = xstate;
    }

    /* do single-step constraint projection */
    if (mCons) {
      auto dst = dstate->as<GeometricStateSpace::StateType>();
      auto pst = pstate->as<GeometricStateSpace::StateType>();
      mCons->project(dst->mState, pst->mState);
      dstate = pstate;
    }

    if (si_->checkMotion(nmotion->state, dstate)) {
      /* create a motion */
      Motion *motion = new Motion(si_);
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;

      mNN->add(motion);
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
  if (solution == nullptr) {
    solution = approxsol;
    approximate = true;
  }

  if (solution != nullptr) {
    mLastGoalMotion = solution;

    /* construct the solution path */
    std::vector<Motion *> mpath;
    while (solution != nullptr) {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    /* set the solution path */
    ::ompl::geometric::PathGeometric *path =
        new ::ompl::geometric::PathGeometric(si_);
    for (int i = mpath.size() - 1; i >= 0; --i)
      path->append(mpath[i]->state);
    pdef_->addSolutionPath(::ompl::base::PathPtr(path), approximate, approxdif);
    solved = true;
  }

  si_->freeState(xstate);
  si_->freeState(pstate);
  if (rmotion->state)
    si_->freeState(rmotion->state);
  delete rmotion;

  return ::ompl::base::PlannerStatus(solved, approximate);
}

//=============================================================================
::ompl::base::PlannerStatus CRRT::solve(double solveTime) {
  return solve(::ompl::base::timedPlannerTerminationCondition(
      solveTime)); //, std::min(solveTime/100., 0.1)));
}

//=============================================================================
double CRRT::distanceFunction(const Motion *a, const Motion *b) const {
  return si_->distance(a->state, b->state);
}
}
}
}
