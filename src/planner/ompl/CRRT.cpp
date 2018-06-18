#include <limits>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <aikido/planner/ompl/CRRT.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>

namespace aikido {
namespace planner {
namespace ompl {
//==============================================================================
CRRT::CRRT(const ::ompl::base::SpaceInformationPtr& _si) : CRRT(_si, "CRRT")
{
}

//==============================================================================
CRRT::CRRT(
    const ::ompl::base::SpaceInformationPtr& _si, const std::string& _name)
  : ::ompl::base::Planner(_si, _name)
  , mGoalBias(1.0)
  , mMaxDistance(0.05)
  , mLastGoalMotion(nullptr)
  , mCons(nullptr)
  , mMaxStepsize(0.1)
  , mMinStepsize(1e-4)
{

  auto ss
      = ompl_dynamic_pointer_cast<GeometricStateSpace>(si_->getStateSpace());
  if (!ss)
  {
    throw std::invalid_argument(
        "CRRT algorithm requires a GeometricStateSpace");
  }

  specs_.approximateSolutions = true;
  specs_.directed = true;

  Planner::declareParam<double>(
      "range", this, &CRRT::setRange, &CRRT::getRange, "0.:1.:10000.");
  Planner::declareParam<double>(
      "goal_bias", this, &CRRT::setGoalBias, &CRRT::getGoalBias, "0.:.05:1.");
  Planner::declareParam<double>(
      "projection_resolution",
      this,
      &CRRT::setProjectionResolution,
      &CRRT::getProjectionResolution,
      "0.:1.:10000.");
  Planner::declareParam<double>(
      "min_step",
      this,
      &CRRT::setMinStateDifference,
      &CRRT::getMinStateDifference,
      "0.:1.:10000.");
}

//==============================================================================
CRRT::~CRRT()
{
  clear();
}

//==============================================================================
void CRRT::getPlannerData(::ompl::base::PlannerData& _data) const
{
  ::ompl::base::Planner::getPlannerData(_data);

  std::vector<Motion*> motions;
  if (mStartTree)
    mStartTree->list(motions);

  if (mLastGoalMotion)
    _data.addGoalVertex(
        ::ompl::base::PlannerDataVertex(mLastGoalMotion->state));

  for (std::size_t i = 0; i < motions.size(); ++i)
  {
    if (motions[i]->parent == nullptr)
      _data.addStartVertex(::ompl::base::PlannerDataVertex(motions[i]->state));
    else
      _data.addEdge(
          ::ompl::base::PlannerDataVertex(motions[i]->parent->state),
          ::ompl::base::PlannerDataVertex(motions[i]->state));
  }
}

//==============================================================================
void CRRT::clear()
{
  ::ompl::base::Planner::clear();
  mSampler.reset();
  freeMemory();
  if (mStartTree)
    mStartTree->clear();
  mLastGoalMotion = nullptr;
}

//==============================================================================
void CRRT::setGoalBias(double _goalBias)
{
  if (_goalBias < 0.0 || _goalBias > 1.0)
  {
    std::stringstream ss;
    ss << "Invalid value for goal bias: " << _goalBias
       << ". Value must be between 0 and 1.";
    throw std::invalid_argument(ss.str());
  }

  mGoalBias = _goalBias;
}

//==============================================================================
double CRRT::getGoalBias() const
{
  return mGoalBias;
}

//==============================================================================
void CRRT::setRange(double _distance)
{
  if (_distance < 0.0)
  {
    throw std::invalid_argument(
        "Distance must be positive on call to setRange.");
  }
  mMaxDistance = _distance;
}

//==============================================================================
double CRRT::getRange() const
{
  return mMaxDistance;
}

//==============================================================================
void CRRT::setPathConstraint(constraint::ProjectablePtr _projectable)
{
//  mCons = std::move(_projectable);
  mCons = nullptr;
}

//==============================================================================
void CRRT::setProjectionResolution(double _resolution)
{
  mMaxStepsize = _resolution;
}

//==============================================================================
double CRRT::getProjectionResolution() const
{
  return mMaxStepsize;
}

//==============================================================================
void CRRT::setMinStateDifference(double _mindist)
{
  mMinStepsize = _mindist;
}

//==============================================================================
double CRRT::getMinStateDifference() const
{
  return mMinStepsize;
}

//==============================================================================
void CRRT::setup()
{
  ::ompl::base::Planner::setup();
  ::ompl::tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(mMaxDistance);

  if (!mStartTree)
    mStartTree.reset(new ::ompl::NearestNeighborsGNAT<Motion*>);

  mStartTree->setDistanceFunction(
      ompl_bind(
          &CRRT::distanceFunction,
          this,
          OMPL_PLACEHOLDER(_1),
          OMPL_PLACEHOLDER(_2)));
}

//==============================================================================
void CRRT::freeMemory()
{
  if (mStartTree)
  {
    std::vector<Motion*> motions;
    mStartTree->list(motions);
    for (std::size_t i = 0; i < motions.size(); ++i)
    {
      if (motions[i]->state)
      {
        si_->freeState(motions[i]->state);
      }
      delete motions[i];
    }
  }
}

//==============================================================================
::ompl::base::PlannerStatus CRRT::solve(
    const ::ompl::base::PlannerTerminationCondition& _ptc)
{
  checkValidity();
  ::ompl::base::Goal* goal = pdef_->getGoal().get();
  ::ompl::base::GoalSampleableRegion* goalSampleable
      = dynamic_cast<::ompl::base::GoalSampleableRegion*>(goal);

  while (const ::ompl::base::State* st = pis_.nextStart())
  {
    Motion* motion = new Motion(si_);
    si_->copyState(motion->state, st);
    mStartTree->add(motion);
  }

  if (mStartTree->size() == 0)
  {
    return ::ompl::base::PlannerStatus::INVALID_START;
  }

  if (!mSampler)
    mSampler = si_->allocStateSampler();

  Motion* solution = nullptr;
  Motion* approxsol = nullptr;
  double approxdif = std::numeric_limits<double>::infinity();
  auto rmotion = std::unique_ptr<Motion>(new Motion(si_));
  ::ompl::base::State* rstate = rmotion->state;
  ::ompl::base::State* xstate = si_->allocState(); /* temp state */

  bool foundgoal = false;
  while (_ptc == false)
  {
    /* sample random state (with goal biasing) */
    if (goalSampleable && mRng.uniform01() < mGoalBias
        && goalSampleable->canSample())
      goalSampleable->sampleGoal(rstate);
    else
      mSampler->sampleUniform(rstate);

    // Continue on invalid sample
    if (!si_->isValid(rstate))
    {
      continue;
    }

    /* find closest state in the tree */
    Motion* nmotion = mStartTree->nearest(rmotion.get());

    /* Perform a constrained extension */
    double bestdist = std::numeric_limits<double>::infinity();
    Motion* bestmotion = constrainedExtend(
        _ptc,
        mStartTree,
        nmotion,
        rmotion->state,
        xstate,
        goal,
        false,
        bestdist,
        foundgoal);
    if (foundgoal)
    {
      solution = bestmotion;
      break;
    }
    else if (bestdist < approxdif)
    {
      approxdif = bestdist;
      approxsol = bestmotion;
    }
  }

  bool solved = false;
  bool approximate = false;
  if (solution == nullptr)
  {
    solution = approxsol;
    approximate = true;
  }

  if (solution != nullptr)
  {
    mLastGoalMotion = solution;

    /* construct the solution path */
    std::vector<Motion*> mpath;
    while (solution != nullptr)
    {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    /* set the solution path */
    auto path = ompl_make_shared<::ompl::geometric::PathGeometric>(si_);
    for (int i = mpath.size() - 1; i >= 0; --i)
    {
      path->append(mpath[i]->state);
      std::cout << "hello" << std::endl;
      std::cout << si_->getStateSpace()->as<::ompl::base::CompoundStateSpace>()->getSubspaceCount();
//      double *Uvals = (mpath[i]->state)->as<::ompl::base::CompoundStateSpace::StateType>()->values;
    }
    pdef_->addSolutionPath(path, approximate, approxdif);
    solved = true;
  }

  si_->freeState(xstate);
  if (rmotion->state)
    si_->freeState(rmotion->state);

  return ::ompl::base::PlannerStatus(solved, approximate);
}

//==============================================================================
CRRT::Motion* CRRT::constrainedExtend(
    const ::ompl::base::PlannerTerminationCondition& ptc,
    TreeData& tree,
    Motion* nmotion,
    ::ompl::base::State* gstate,
    ::ompl::base::State* xstate,
    ::ompl::base::Goal* goal,
    bool returnlast,
    double& dist,
    bool& foundgoal)
{

  // Set up the current parent motion
  Motion* cmotion = nmotion;
  dist = std::numeric_limits<double>::infinity();
  Motion* bestmotion = nmotion;

  // Compute the current and previous distance to the goal state
  double prevDistToTarget = std::numeric_limits<double>::infinity();
  double distToTarget = si_->distance(cmotion->state, gstate);

  // Loop while time remaining
  foundgoal = false;
  while (ptc == false)
  {

    if (distToTarget == 0 || distToTarget - prevDistToTarget >= -mMinStepsize)
    {
      // reached target or not making progress
      break;
    }

    // Take a step towards the goal state
    double stepLength
        = std::min(mMaxDistance, std::min(mMaxStepsize, distToTarget));
    si_->getStateSpace()->interpolate(
        cmotion->state, gstate, stepLength / distToTarget, xstate);

    if (mCons)
    {
      // Project the endpoint of the step
      auto xst = xstate->as<GeometricStateSpace::StateType>();
      if (!mCons->project(xst->mState))
      {
        // Can't project back to constraint anymore, return
        break;
      }
    }

    if (si_->checkMotion(cmotion->state, xstate))
    {
      // Add the motion to the tree
      Motion* motion = new Motion(si_);
      si_->copyState(motion->state, xstate);
      motion->parent = cmotion;
      tree->add(motion);

      cmotion = motion;
      double newdist = 0.0;
      bool satisfied = goal->isSatisfied(motion->state, &newdist);
      if (satisfied)
      {
        dist = newdist;
        bestmotion = motion;
        foundgoal = true;
        break;
      }
      if (newdist < dist)
      {
        dist = newdist;
        bestmotion = motion;
      }
      if (returnlast)
      {
        // Always make the "bestmotion" the current motion
        bestmotion = motion;
      }
    }
    else
    {
      // Extension failed validity check
      break;
    }
    prevDistToTarget = distToTarget;
    distToTarget = si_->distance(cmotion->state, gstate);
  }

  return bestmotion;
}

//==============================================================================
::ompl::base::PlannerStatus CRRT::solve(double solveTime)
{
  return solve(
      ::ompl::base::timedPlannerTerminationCondition(
          solveTime)); //, std::min(solveTime/100., 0.1)));
}

//==============================================================================
double CRRT::distanceFunction(const Motion* a, const Motion* b) const
{
  return si_->distance(a->state, b->state);
}
}
}
}
