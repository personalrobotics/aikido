#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <aikido/planner/ompl/BackwardCompatibility.hpp>
#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>

namespace aikido {
namespace planner {
namespace ompl {

//==============================================================================
CRRTConnect::CRRTConnect(const ::ompl::base::SpaceInformationPtr& _si)
  : CRRT(_si, "CRRTConnect"), mConnectionRadius(1e-4)
{

  specs_.recognizedGoal = ::ompl::base::GOAL_SAMPLEABLE_REGION;
  Planner::declareParam<double>(
      "connectionRadius",
      this,
      &CRRTConnect::setConnectionRadius,
      &CRRTConnect::getConnectionRadius,
      "0.:1.:10000.");
  mConnectionPoint = std::make_pair<::ompl::base::State*, ::ompl::base::State*>(
      nullptr, nullptr);
}

//==============================================================================
CRRTConnect::~CRRTConnect()
{
  clear();
}

//==============================================================================
void CRRTConnect::setup()
{
  Planner::setup();
  ::ompl::tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(mMaxDistance);

  if (!mStartTree)
    mStartTree.reset(new ::ompl::NearestNeighborsGNAT<Motion*>);
  if (!mGoalTree)
    mGoalTree.reset(new ::ompl::NearestNeighborsGNAT<Motion*>);

  mStartTree->setDistanceFunction(
      ompl_bind(
          &CRRTConnect::distanceFunction,
          this,
          OMPL_PLACEHOLDER(_1),
          OMPL_PLACEHOLDER(_2)));
  mGoalTree->setDistanceFunction(
      ompl_bind(
          &CRRTConnect::distanceFunction,
          this,
          OMPL_PLACEHOLDER(_1),
          OMPL_PLACEHOLDER(_2)));
}

//==============================================================================
void CRRTConnect::freeMemory()
{
  CRRT::freeMemory();

  if (mGoalTree)
  {
    std::vector<Motion*> motions;
    mGoalTree->list(motions);
    for (std::size_t i = 0; i < motions.size(); ++i)
    {
      if (motions[i]->state)
        si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

//==============================================================================
void CRRTConnect::clear()
{
  CRRT::clear();
  if (mGoalTree)
    mGoalTree->clear();
  mConnectionPoint = std::make_pair<::ompl::base::State*, ::ompl::base::State*>(
      nullptr, nullptr);
}

//==============================================================================
void CRRTConnect::setConnectionRadius(double radius)
{
  mConnectionRadius = radius;
}

//==============================================================================
double CRRTConnect::getConnectionRadius() const
{
  return mConnectionRadius;
}

//==============================================================================
::ompl::base::PlannerStatus CRRTConnect::solve(
    const ::ompl::base::PlannerTerminationCondition& ptc)
{
  checkValidity();

  ::ompl::base::GoalSampleableRegion* goal
      = dynamic_cast<::ompl::base::GoalSampleableRegion*>(
          pdef_->getGoal().get());

  if (!goal)
  {
    return ::ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
  }

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

  if (!goal->couldSample())
  {
    return ::ompl::base::PlannerStatus::INVALID_GOAL;
  }

  if (!mSampler)
    mSampler = si_->allocStateSampler();

  // Extra state used during tree extensions
  ::ompl::base::State* xstate = si_->allocState();

  auto rmotion = std::unique_ptr<Motion>(new Motion(si_));
  ::ompl::base::State* rstate = rmotion->state;

  bool startTree = true;
  bool solved = false;
  bool foundgoal = false;

  while (ptc == false)
  {
    TreeData& tree = startTree ? mStartTree : mGoalTree;
    TreeData& otherTree = startTree ? mGoalTree : mStartTree;
    startTree = !startTree;

    if (mGoalTree->size() == 0
        || pis_.getSampledGoalsCount() < mGoalTree->size() / 2)
    {
      while (ptc == false)
      {
        const ::ompl::base::State* st = pis_.nextGoal(ptc);
        if (si_->isValid(st))
        {
          Motion* motion = new Motion(si_);
          si_->copyState(motion->state, st);
          mGoalTree->add(motion);
        }
        if (mGoalTree->size() > 0)
          break;
      }
    }

    // Sample a random state
    mSampler->sampleUniform(rstate);
    if (!si_->isValid(rstate))
      continue;

    // Find closest state in tree
    Motion* nmotion = tree->nearest(rmotion.get());

    // Grow one tree toward the random sample
    double bestdist = std::numeric_limits<double>::infinity();
    Motion* lastmotion = constrainedExtend(
        ptc,
        tree,
        nmotion,
        rmotion->state,
        xstate,
        goal,
        true,
        bestdist,
        foundgoal);

    if (lastmotion == nmotion)
    {
      // trapped
      continue;
    }

    // Now grow the other tree
    nmotion = otherTree->nearest(lastmotion);
    Motion* newmotion = constrainedExtend(
        ptc,
        otherTree,
        nmotion,
        lastmotion->state,
        xstate,
        goal,
        true,
        bestdist,
        foundgoal);

    Motion* startMotion = startTree ? newmotion : lastmotion;
    Motion* goalMotion = startTree ? lastmotion : newmotion;

    double treedist = si_->distance(newmotion->state, lastmotion->state);
    if (treedist <= mConnectionRadius)
    {
      if (treedist < 1e-6)
      {
        // The start and goal trees hit the same point, remove one of them
        // to avoid having a duplicate state on the path
        if (startMotion->parent)
          startMotion = startMotion->parent;
        else
          goalMotion = goalMotion->parent;
      }

      mConnectionPoint = std::make_pair(startMotion->state, goalMotion->state);

      /* construct the solution path */
      Motion* solution = startMotion;
      std::vector<Motion*> mpath1;
      while (solution != nullptr)
      {
        mpath1.push_back(solution);
        solution = solution->parent;
      }

      solution = goalMotion;
      std::vector<Motion*> mpath2;
      while (solution != nullptr)
      {
        mpath2.push_back(solution);
        solution = solution->parent;
      }

      // Double check that the start and goal pair are valid
      if (mpath1.size() > 0 && mpath2.size() > 0)
      {
        if (!goal->isStartGoalPairValid(
                mpath1.front()->state, mpath2.back()->state))
          continue;
      }

      auto path = ompl_make_shared<::ompl::geometric::PathGeometric>(si_);
      path->getStates().reserve(mpath1.size() + mpath2.size());
      for (int i = mpath1.size() - 1; i >= 0; --i)
        path->append(mpath1[i]->state);
      for (std::size_t i = 0; i < mpath2.size(); ++i)
        path->append(mpath2[i]->state);

      pdef_->addSolutionPath(path, false, 0.0);
      solved = true;
      break;
    }
  }

  si_->freeState(xstate);
  si_->freeState(rstate);

  return solved ? ::ompl::base::PlannerStatus::EXACT_SOLUTION
                : ::ompl::base::PlannerStatus::TIMEOUT;
}

//==============================================================================
void CRRTConnect::getPlannerData(::ompl::base::PlannerData& data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion*> motions;
  if (mStartTree)
    mStartTree->list(motions);

  for (std::size_t i = 0; i < motions.size(); ++i)
  {
    if (motions[i]->parent == nullptr)
      data.addStartVertex(
          ::ompl::base::PlannerDataVertex(motions[i]->state, 1));
    else
    {
      data.addEdge(
          ::ompl::base::PlannerDataVertex(motions[i]->parent->state, 1),
          ::ompl::base::PlannerDataVertex(motions[i]->state, 1));
    }
  }

  motions.clear();
  if (mGoalTree)
    mGoalTree->list(motions);

  for (std::size_t i = 0; i < motions.size(); ++i)
  {
    if (motions[i]->parent == nullptr)
    {
      data.addGoalVertex(::ompl::base::PlannerDataVertex(motions[i]->state, 2));
    }
    else
    {
      // The edges in the goal tree are reversed to be consistent with start
      // tree
      data.addEdge(
          ::ompl::base::PlannerDataVertex(motions[i]->state, 2),
          ::ompl::base::PlannerDataVertex(motions[i]->parent->state, 2));
    }
  }

  // Add the edge connecting the two trees
  data.addEdge(
      data.vertexIndex(mConnectionPoint.first),
      data.vertexIndex(mConnectionPoint.second));
}

} // namespace ompl
} // namespace planner
} // namespace aikido