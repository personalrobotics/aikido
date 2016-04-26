#include <aikido/planner/ompl/CRRTConnect.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>

namespace aikido {
namespace planner
{
namespace ompl
{

//=============================================================================
CRRTConnect::CRRTConnect(const ::ompl::base::SpaceInformationPtr &si)
    : ::ompl::base::Planner(si, "CRRTConnect")
{
  specs_.recognizedGoal = ::ompl::base::GOAL_SAMPLEABLE_REGION;
  specs_.directed = true;

  maxDistance_ = 0.0;

  Planner::declareParam<double>("range", this, &CRRTConnect::setRange,
                                &CRRTConnect::getRange, "0.:1.:10000.");
  connectionPoint_ =
      std::make_pair<::ompl::base::State *, ::ompl::base::State *>(NULL, NULL);

  cons_ = nullptr;
}

//=============================================================================
CRRTConnect::~CRRTConnect(void) { freeMemory(); }

//=============================================================================
void CRRTConnect::setup(void)
{
  Planner::setup();
  ::ompl::tools::SelfConfig sc(si_, getName());
  sc.configurePlannerRange(maxDistance_);

  if (!tStart_)
    tStart_.reset(
        ::ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(
            si_->getStateSpace()));
  if (!tGoal_)
    tGoal_.reset(
        ::ompl::tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(
            si_->getStateSpace()));
  tStart_->setDistanceFunction(
      boost::bind(&CRRTConnect::distanceFunction, this, _1, _2));
  tGoal_->setDistanceFunction(
      boost::bind(&CRRTConnect::distanceFunction, this, _1, _2));
}

//=============================================================================
void CRRTConnect::freeMemory(void)
{
  std::vector<Motion *> motions;

  if (tStart_) {
    tStart_->list(motions);
    for (unsigned int i = 0; i < motions.size(); ++i) {
      if (motions[i]->state) si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }

  if (tGoal_) {
    tGoal_->list(motions);
    for (unsigned int i = 0; i < motions.size(); ++i) {
      if (motions[i]->state) si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

void CRRTConnect::clear(void)
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (tStart_) tStart_->clear();
  if (tGoal_) tGoal_->clear();
  connectionPoint_ =
      std::make_pair<::ompl::base::State *, ::ompl::base::State *>(NULL, NULL);
}

//=============================================================================
void CRRTConnect::setRange(double distance) 
{
  maxDistance_ = distance;
}

//=============================================================================
double CRRTConnect::getRange() const 
{ 
  return maxDistance_;
}

//=============================================================================
void CRRTConnect::setTrajectoryWideConstraint(
    constraint::ProjectablePtr _projectable)
{
  cons_ = std::move(_projectable);
}

//=============================================================================
CRRTConnect::GrowState CRRTConnect::growTree(TreeData &tree,
                                             TreeGrowingInfo &tgi,
                                             Motion *rmotion)
{
  /* find closest state in the tree */
  Motion *nmotion = tree->nearest(rmotion);

  /* assume we can reach the state we go towards */
  bool reach = true;

  /* find state to add (initially unconstrained) */
  ::ompl::base::State *dstate = rmotion->state;
  double d = si_->distance(nmotion->state, rmotion->state);
  if (d > maxDistance_) {
    si_->getStateSpace()->interpolate(nmotion->state, rmotion->state,
                                      maxDistance_ / d, tgi.xstate);
    dstate = tgi.xstate;
    reach = false;
  }

  /* do single-step constraint projection */
  if (cons_) {
    // TODO: Check if the constraint is satisfied before doing this
    //  to save the extra distance computation
    auto dst = dstate->as<GeometricStateSpace::StateType>();
    auto pst = tgi.pstate->as<GeometricStateSpace::StateType>();
    cons_->project(dst->mState, pst->mState);
    dstate = tgi.pstate;

    double d_new = si_->distance(dstate, rmotion->state);
    if (d - d_new < 0.0001) return TRAPPED;

    reach = false;
  }

  // if we are in the start tree, we just check the motion like we normally do;
  // if we are in the goal tree, we need to check the motion in reverse, but
  // checkMotion() assumes the first state it receives as argument is valid,
  // so we check that one first
  bool validMotion = tgi.start ?
                         si_->checkMotion(nmotion->state, dstate) :
                         si_->getStateValidityChecker()->isValid(dstate)
                             && si_->checkMotion(dstate, nmotion->state);

  if (validMotion) {
    /* create a motion */
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, dstate);
    motion->parent = nmotion;
    motion->root = nmotion->root;
    tgi.xmotion = motion;

    tree->add(motion);
    if (reach)
      return REACHED;
    else
      return ADVANCED;
  } else
    return TRAPPED;
}

//=============================================================================
::ompl::base::PlannerStatus CRRTConnect::solve(
    const ::ompl::base::PlannerTerminationCondition &ptc)
{
  checkValidity();
  ::ompl::base::GoalSampleableRegion *goal =
      dynamic_cast<::ompl::base::GoalSampleableRegion *>(
          pdef_->getGoal().get());

  if (!goal) {
    // OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
    return ::ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
  }

  while (const ::ompl::base::State *st = pis_.nextStart()) {
    // if (!satisfiesConstraint(st)) {
    //   //OMPL_WARN("a start state does not meet the constraint! ignoring ...");
    //   continue;
    // }
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    motion->root = motion->state;
    tStart_->add(motion);
  }

  if (tStart_->size() == 0) {
    //OMPL_ERROR("%s: Motion planning start tree could not be initialized!",
    //          getName().c_str());
    return ::ompl::base::PlannerStatus::INVALID_START;
  }

  if (!goal->couldSample()) {
    //OMPL_ERROR("%s: Insufficient states in sampleable goal region",
    //           getName().c_str());
    return ::ompl::base::PlannerStatus::INVALID_GOAL;
  }

  if (!sampler_) sampler_ = si_->allocStateSampler();

  //OMPL_INFORM("%s: Starting with %d states", getName().c_str(),
  //            (int)(tStart_->size() + tGoal_->size()));

  TreeGrowingInfo tgi;
  tgi.xstate = si_->allocState();
  tgi.pstate = si_->allocState();

  Motion *rmotion = new Motion(si_);
  ::ompl::base::State *rstate = rmotion->state;
  bool startTree = true;
  bool solved = false;

  while (ptc == false) {
    TreeData &tree = startTree ? tStart_ : tGoal_;
    tgi.start = startTree;
    startTree = !startTree;
    TreeData &otherTree = startTree ? tStart_ : tGoal_;

    if (tGoal_->size() == 0) {
      while (ptc == false) {
        const ::ompl::base::State *st = pis_.nextGoal(ptc);
        if (!si_->isValid(st)) {
          //OMPL_ERROR("%s: Unable to sample any valid states for goal tree",
          //           getName().c_str());
            continue;
        }
        // if (!satisfiesConstraint(st)) {
        //   //OMPL_WARN("a goal state does not meet the constraint! ignoring ...");
        //   continue;
        // }
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tGoal_->add(motion);
        break;
      }
    } else if (pis_.getSampledGoalsCount() < tGoal_->size() / 2) {
      const ::ompl::base::State *st = pis_.nextGoal();
      if (si_->isValid(st)) {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tGoal_->add(motion);
      }
    }

    /* sample random state */
    sampler_->sampleUniform(rstate);

    GrowState gs = growTree(tree, tgi, rmotion);

    if (gs != TRAPPED) {
      /* remember which motion was just added */
      Motion *addedMotion = tgi.xmotion;

      /* attempt to connect trees */

      /* if reached, it means we used rstate directly, no need top copy again */
      if (gs != REACHED) si_->copyState(rstate, tgi.xstate);

      /* if constrained, then we always got to pstate */
      if (cons_) si_->copyState(rstate, tgi.pstate);

      GrowState gsc = ADVANCED;
      tgi.start = startTree;
      while (gsc == ADVANCED) gsc = growTree(otherTree, tgi, rmotion);

      Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
      Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

      /* if we connected the trees in a valid way (start and goal pair is
       * valid)*/
      if (gsc == REACHED
          && goal->isStartGoalPairValid(startMotion->root, goalMotion->root)) {
        // it must be the case that either the start tree or the goal tree has
        // made some progress
        // so one of the parents is not NULL. We go one step 'back' to avoid
        // having a duplicate state
        // on the solution path
        if (startMotion->parent)
          startMotion = startMotion->parent;
        else
          goalMotion = goalMotion->parent;

        connectionPoint_ =
            std::make_pair(startMotion->state, goalMotion->state);

        /* construct the solution path */
        Motion *solution = startMotion;
        std::vector<Motion *> mpath1;
        while (solution != NULL) {
          mpath1.push_back(solution);
          solution = solution->parent;
        }

        solution = goalMotion;
        std::vector<Motion *> mpath2;
        while (solution != NULL) {
          mpath2.push_back(solution);
          solution = solution->parent;
        }

        ::ompl::geometric::PathGeometric *path =
              new ::ompl::geometric::PathGeometric(si_);
        path->getStates().reserve(mpath1.size() + mpath2.size());
        for (int i = mpath1.size() - 1; i >= 0; --i)
          path->append(mpath1[i]->state);
        for (unsigned int i = 0; i < mpath2.size(); ++i)
          path->append(mpath2[i]->state);

        pdef_->addSolutionPath(::ompl::base::PathPtr(path), false, 0.0);
        solved = true;
        break;
      }
    }
  }

  si_->freeState(tgi.xstate);
  si_->freeState(tgi.pstate);
  si_->freeState(rstate);
  delete rmotion;

  //OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(),
  //            tStart_->size() + tGoal_->size(), tStart_->size(),
  //            tGoal_->size());

  return solved ? ::ompl::base::PlannerStatus::EXACT_SOLUTION :
      ::ompl::base::PlannerStatus::TIMEOUT;
}

//=============================================================================
void CRRTConnect::getPlannerData(::ompl::base::PlannerData &data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion *> motions;
  if (tStart_) tStart_->list(motions);

  for (unsigned int i = 0; i < motions.size(); ++i) {
    if (motions[i]->parent == NULL)
      data.addStartVertex(
          ::ompl::base::PlannerDataVertex(motions[i]->state, 1));
    else {
      data.addEdge(
          ::ompl::base::PlannerDataVertex(motions[i]->parent->state, 1),
          ::ompl::base::PlannerDataVertex(motions[i]->state, 1));
    }
  }

  motions.clear();
  if (tGoal_) tGoal_->list(motions);

  for (unsigned int i = 0; i < motions.size(); ++i) {
    if (motions[i]->parent == NULL)
        data.addGoalVertex(::ompl::base::PlannerDataVertex(motions[i]->state, 2));
    else {
      // The edges in the goal tree are reversed to be consistent with start
      // tree
      data.addEdge(::ompl::base::PlannerDataVertex(motions[i]->state, 2),
                   ::ompl::base::PlannerDataVertex(motions[i]->parent->state, 2));
    }
  }

  // Add the edge connecting the two trees
  data.addEdge(data.vertexIndex(connectionPoint_.first),
               data.vertexIndex(connectionPoint_.second));
}
}
}
}
