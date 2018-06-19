#include <chrono>
#include <iostream>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <aikido/planner/kinodynamics/dimt/Params.h>
#include <aikido/planner/kinodynamics/ompl/MyInformedRRTstar.hpp>
#include "ompl/base/Goal.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"

using namespace ompl;
using namespace ompl::geometric;

bool RRT_VERBOSE = false;

//
// MyInformedRRTstar
// This is here mainly for debugging
//

namespace ompl {
namespace geometric {

MyInformedRRTstar::MyInformedRRTstar(const ompl::base::SpaceInformationPtr& si)
  : InformedRRTstar(si)
{
  
  setName("KinoInformedRRTstar");

  mode_ = RANDOM_SAMPLES;
  setTreePruning(false);
  useRejectionSampling_ = false;
  useNewStateRejection_ = false;
  std::cout << " useInformedSampling_ " << useInformedSampling_ << std::endl;
  // infSampler_->sampleUniform(NULL, ompl::base::Cost(100.0));
  // maxDistance_ = 10.0;

  // A hack to approximate an infinite connection radius
  setRewireFactor(10000.);

  setTreePruning(false);
  setNewStateRejection(false);
  setDelayCC(false);
}

void MyInformedRRTstar::initLogFile(
    std::string scenarioName, std::string samplerName, int id)
{
  std::stringstream ss;
  ss << scenarioName.c_str() << "_" << samplerName.c_str() << "_" << id
     << ".csv";
  out_.open(ss.str());

  std::cout << "SAVING FILE TO " << ss.str() << " = " << out_.is_open()
            << std::endl;
}

ompl::base::PlannerStatus MyInformedRRTstar::solve(double solveTime)
{
  if (solveTime < 1.0)
    return solve(ompl::base::timedPlannerTerminationCondition(solveTime));
  return solve(
      ompl::base::timedPlannerTerminationCondition(
          solveTime, std::min(solveTime / 100.0, 0.1)));
}

ompl::base::PlannerStatus MyInformedRRTstar::solveAfterLoadingSamples(
    std::string filename, double solveTime)
{
  mode_ = LOAD_SAMPLES;

  /*
  if(nn_)
  {
      //nn_.reset(new NearestNeighborsLinear<Motion *>());
      nn_.reset(new NearestNeighborsGNAT<Motion*>());
      nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return
  distanceFunction(a, b); });
  }*/

  sampleLoadStream_.open(filename.c_str(), std::ios::in);
  if (sampleLoadStream_.is_open() == false)
  {
    throw std::runtime_error("fail in loading sample file");
  }
  else
  {
    std::cout << "sample file loaded" << std::endl;
  }
  loadedSamplesStr_.clear();
  if (sampleLoadStream_.eof())
  {
    throw std::runtime_error("EOF");
  }
  std::string tmpStr;
  while (std::getline(sampleLoadStream_, tmpStr))
  {
    loadedSamplesStr_.push_back(tmpStr);
    // std::cout << "TEST " << tmpStr.c_str() << std::endl;
  }

  if (solveTime < 1.0)
    return solve(ompl::base::timedPlannerTerminationCondition(solveTime));
  return solve(
      ompl::base::timedPlannerTerminationCondition(
          solveTime, std::min(solveTime / 100.0, 0.1)));
}

ompl::base::PlannerStatus MyInformedRRTstar::solveAndSaveSamples(
    std::string filename, double solveTime)
{
  mode_ = SAVE_SAMPLES;

  /*
  if(nn_)
  {
      //nn_.reset(new NearestNeighborsLinear<Motion *>());
      nn_.reset(new NearestNeighborsGNAT<Motion*>());
      nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return
  distanceFunction(a, b); });
  }*/

  sampleSaveStream_.open(filename.c_str(), std::ios::out);
  if (solveTime < 1.0)
    return solve(ompl::base::timedPlannerTerminationCondition(solveTime));
  return solve(
      ompl::base::timedPlannerTerminationCondition(
          solveTime, std::min(solveTime / 100.0, 0.1)));
}

base::PlannerStatus MyInformedRRTstar::solve(
    const base::PlannerTerminationCondition& ptc)
{
  OMPL_INFORM(
      "%s: YOU ARE CALLING THE RIGHT ONE",
      getName().c_str());

  // std::cout << "Using Correct Informed RRT*" << std::endl;
  checkValidity();
  base::Goal* goal = pdef_->getGoal().get();
  base::GoalSampleableRegion* goal_s
      = dynamic_cast<base::GoalSampleableRegion*>(goal);

  bool symCost = opt_->isSymmetric();

  // Check if there are more starts
  if (pis_.haveMoreStartStates() == true)
  {
    // There are, add them
    while (const base::State* st = pis_.nextStart())
    {
      Motion* motion = new Motion(si_);
      si_->copyState(motion->state, st);
      motion->cost = opt_->identityCost();
      nn_->add(motion);
      startMotions_.push_back(motion);
    }

    // And assure that, if we're using an informed sampler, it's reset
    infSampler_.reset();
  }
  // No else

  if (nn_->size() == 0)
  {
    OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    return base::PlannerStatus::INVALID_START;
  }

  // Allocate a sampler if necessary
  if (!sampler_ && !infSampler_)
  {
    allocSampler();
  }

  OMPL_INFORM(
      "%s: Starting planning with %u states already in datastructure",
      getName().c_str(),
      nn_->size());

  if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_
       || useNewStateRejection_)
      && !si_->getStateSpace()->isMetricSpace())
    OMPL_WARN(
        "%s: The state space (%s) is not metric and as a result the "
        "optimization objective may not satisfy the triangle inequality. "
        "You may need to disable pruning or rejection.",
        getName().c_str(),
        si_->getStateSpace()->getName().c_str());

  const base::ReportIntermediateSolutionFn intermediateSolutionCallback
      = pdef_->getIntermediateSolutionCallback();

  Motion* solution = lastGoalMotion_;

  Motion* approximation = nullptr;
  double approximatedist = std::numeric_limits<double>::infinity();
  bool sufficientlyShort = false;

  Motion* rmotion = new Motion(si_);
  base::State* rstate = rmotion->state;
  base::State* xstate = si_->allocState();

  std::vector<Motion*> nbh;

  std::vector<base::Cost> costs;
  std::vector<base::Cost> incCosts;
  std::vector<std::size_t> sortedCostIndices;

  std::vector<int> valid;
  unsigned int rewireTest = 0;
  unsigned int statesGenerated = 0;

  if (solution)
    OMPL_INFORM(
        "%s: Starting planning with existing solution of cost %.5f",
        getName().c_str(),
        solution->cost.value());

  /*
  if (useKNearest_)

      OMPL_INFORM("%s: k_rrt_ %u ->> Initial k-nearest value of %u",
  getName().c_str(),
                  k_rrt_,
                  (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() +
  1u))));
  else
      OMPL_INFORM(
          "%s: Initial rewiring radius of %.2f", getName().c_str(),
          std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() +
  1u)) / ((double)(nn_->size() + 1u)),
                                                   1 /
  (double)(si_->getStateDimension()))));
  */

  // our functor for sorting nearest neighbors
  CostIndexCompare compareFn(costs, *opt_);

  samplesGeneratedNum_ = 0;

  std::chrono::high_resolution_clock::time_point startTime
      = std::chrono::high_resolution_clock::now();
  while (ptc == false)
  {
    /* DISABLE
    //first iteration, try to explicitly connect start to goal
    if (iterations_ == 0)
    {
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() &&
    goal_s->canSample())
            goal_s->sampleGoal(rstate);

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion); //this is the start

        if (si_->checkMotion(nmotion->state, rstate))
        {
            OMPL_INFORM("TRIVIAL PROBLEM< CONNECT START TO GOAL OPTIMALY---NOT
    RUNNING PLANNER ");
            return base::PlannerStatus(false, false);
        }
    }
    */

    iterations_++;

    // OMPL_INFORM(" %u-th iteration ", iterations_);

    // sample random state (with goal biasing)
    // Goal samples are only sampled until maxSampleCount() goals are in the
    // tree, to prohibit duplicate goal states.
    if (goal_s && goalMotions_.size() < goal_s->maxSampleCount()
        && rng_.uniform01() < goalBias_
        && goal_s->canSample()
        && (mode_ != LOAD_SAMPLES || opt_->isFinite(bestCost_)))
    {
      goal_s->sampleGoal(rstate);
      if (mode_ == SAVE_SAMPLES)
      {
        // append sample to file
        std::string stateStr = fromState(rstate);
        sampleSaveStream_ << stateStr << std::endl;
      }
    }
    else
    {
      // Attempt to generate a sample, if we fail (e.g., too many rejection
      // attempts), skip the remainder of this loop and return to try again
      // std::cout << "Printing state before: ";
      // print_out_states(rstate);
      if (opt_->isFinite(bestCost_))
      {
        if (!sampleUniform(rstate))
        {
          continue;
        }
        samplesGeneratedNum_++;
      }
      else //(opt_->isFinite(bestCost_) == false)
      {
        if (mode_ == SAVE_SAMPLES)
        {
          if (!sampleUniform(rstate))
          {
            continue;
          }
          else
          {
            // append sample to file
            std::string stateStr = fromState(rstate);
            sampleSaveStream_ << stateStr << std::endl;
          }
        }
        else if (mode_ == LOAD_SAMPLES)
        {
          if (loadedSamplesStr_.size() > 0)
          {

            std::string stateStr = loadedSamplesStr_.front();
            loadedSamplesStr_.pop_front();
            bool success = toState(stateStr, rstate);
            if (success == false)
            {
              std::cout << "FAIL " << std::endl;
            }
            // std::cout << stateStr.c_str() << std::endl;
          }
          else
          {
            throw std::runtime_error("NO SAMPLE LEFT");
          }
        }
        else
        {
          if (!sampleUniform(rstate))
          {
            continue;
          }
        }
      }
    }

    // find closest state in the tree
    Motion* nmotion = nn_->nearest(rmotion);

    if (intermediateSolutionCallback
        && si_->equalStates(nmotion->state, rstate))
      continue;

    base::State* dstate = rstate;

    // find state to add to the tree
    /* FOLLOWING HRS PAPER WE USE oo EXTENSION
    double d = si_->distance(nmotion->state, rstate);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ /
    d, xstate);
        dstate = xstate;
    }
    */

    // Check if the motion between the nearest state and the state to add is
    // valid
    if (si_->checkMotion(nmotion->state, dstate))
    {
      // std::cout << "check motion succeed" << std::endl;
      // create a motion
      Motion* motion = new Motion(si_);
      si_->copyState(motion->state, dstate);
      motion->parent = nmotion;
      motion->incCost = opt_->motionCost(nmotion->state, motion->state);
      motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

      // Find nearby neighbors of the new motion
      // getNeighbors(motion, nbh);
      nbh.clear();
      nn_->list(nbh);

      rewireTest += nbh.size();
      ++statesGenerated;

      // cache for distance computations
      //
      // Our cost caches only increase in size, so they're only
      // resized if they can't fit the current neighborhood
      if (costs.size() < nbh.size())
      {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
      }

      // cache for motion validity (only useful in a symmetric space)
      //
      // Our validity caches only increase in size, so they're
      // only resized if they can't fit the current neighborhood
      if (valid.size() < nbh.size())
        valid.resize(nbh.size());
      std::fill(valid.begin(), valid.begin() + nbh.size(), 0);

      // Finding the nearest neighbor to connect to
      // By default, neighborhood states are sorted by cost, and collision
      // checking
      // is performed in increasing order of cost
      if (delayCC_)
      {
        // calculate all costs and distances
        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
          incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
          costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
        }

        // sort the nodes
        //
        // we're using index-value pairs so that we can get at
        // original, unsorted indices
        for (std::size_t i = 0; i < nbh.size(); ++i)
          sortedCostIndices[i] = i;
        std::sort(
            sortedCostIndices.begin(),
            sortedCostIndices.begin() + nbh.size(),
            compareFn);

        // collision check until a valid motion is found
        //
        // ASYMMETRIC CASE: it's possible that none of these
        // neighbors are valid. This is fine, because motion
        // already has a connection to the tree through
        // nmotion (with populated cost fields!).
        for (std::vector<std::size_t>::const_iterator i
             = sortedCostIndices.begin();
             i != sortedCostIndices.begin() + nbh.size();
             ++i)
        {
          if (nbh[*i] == nmotion
              || si_->checkMotion(nbh[*i]->state, motion->state))
          {
            motion->incCost = incCosts[*i];
            motion->cost = costs[*i];
            motion->parent = nbh[*i];
            valid[*i] = 1;
            break;
          }
          else
          {
            valid[*i] = -1;
          }
        }
      }
      else // if not delayCC
      {
        if (RRT_VERBOSE)
          std::cout << "Not valid state" << std::endl;

        motion->incCost = opt_->motionCost(nmotion->state, motion->state);
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
        // find which one we connect the new state to
        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
          if (nbh[i] != nmotion)
          {
            incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
            costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
            if (opt_->isCostBetterThan(costs[i], motion->cost))
            {
              if (si_->checkMotion(nbh[i]->state, motion->state))
              {
                motion->incCost = incCosts[i];
                motion->cost = costs[i];
                motion->parent = nbh[i];
                valid[i] = 1;
              }
              else
                valid[i] = -1;
            }
          }
          else
          {
            incCosts[i] = motion->incCost;
            costs[i] = motion->cost;
            valid[i] = 1;
          }
        }
      }

      if (useNewStateRejection_)
      {
        if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))
        {
          nn_->add(motion);
          motion->parent->children.push_back(motion);
        }
        else // If the new motion does not improve the best cost it is
             // ignored.
        {
          si_->freeState(motion->state);
          delete motion;
          continue;
        }
      }
      else
      {
        // add motion to the tree
        nn_->add(motion);
        motion->parent->children.push_back(motion);
      }

      bool checkForSolution = false;
      for (std::size_t i = 0; i < nbh.size(); ++i)
      {
        if (nbh[i] != motion->parent)
        {
          base::Cost nbhIncCost;
          if (symCost)
            nbhIncCost = incCosts[i];
          else
            nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
          base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
          if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
          {
            bool motionValid;
            if (valid[i] == 0)
            {
              motionValid = si_->checkMotion(motion->state, nbh[i]->state);
            }
            else
            {
              motionValid = (valid[i] == 1);
            }

            if (motionValid)
            {
              // Remove this node from its parent list
              removeFromParent(nbh[i]);

              // Add this node to the new parent
              nbh[i]->parent = motion;
              nbh[i]->incCost = nbhIncCost;
              nbh[i]->cost = nbhNewCost;
              nbh[i]->parent->children.push_back(nbh[i]);

              // Update the costs of the node's children
              updateChildCosts(nbh[i]);

              checkForSolution = true;
            }
          }
        }
      }

      // Add the new motion to the goalMotion_ list, if it satisfies the goal
      double distanceFromGoal = 0.0;
      if (goal->isSatisfied(motion->state, &distanceFromGoal))
      {
        goalMotions_.push_back(motion);
        checkForSolution = true;
      }

      // Checking for solution or iterative improvement
      if (checkForSolution)
      {
        bool updatedSolution = false;
        for (std::size_t i = 0; i < goalMotions_.size(); ++i)
        {
          if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost_))
          {
            if (opt_->isFinite(bestCost_) == false)
            {
              OMPL_INFORM(
                  "%s: Found an initial solution with a cost of %.2f "
                  "in %u iterations (%u vertices in the graph)",
                  getName().c_str(),
                  goalMotions_[i]->cost.value(),
                  iterations_,
                  nn_->size());

              if (mode_ == SAVE_SAMPLES)
              {
                sampleSaveStream_.close();
                return base::PlannerStatus(true, false);
              }
              else if (mode_ == LOAD_SAMPLES)
              {
                // close file stream
                sampleLoadStream_.close();

                // switch to apx NN data structure
                auto tmp = aikido::planner::ompl::
                    ompl_make_shared<NearestNeighborsSqrtApprox<Motion*>>();
                tmp->setDistanceFunction(
                    [this](const Motion* a, const Motion* b) {
                      return distanceFunction(a, b);
                    });
                std::vector<Motion*> motions;
                nn_->list(motions);
                for (std::size_t j(0); j < motions.size(); ++j)
                  tmp->add(motions[j]);
                nn_ = tmp;

                startTime = std::chrono::high_resolution_clock::now();
              }
            }
            else
            {
              OMPL_INFORM(
                  "%s: Found an better solution with a cost of %.2f "
                  "in %u iterations (%u vertices in the graph)",
                  getName().c_str(),
                  goalMotions_[i]->cost.value(),
                  iterations_,
                  nn_->size());
            }
            if (out_.is_open())
            {
              std::chrono::high_resolution_clock::time_point currentTime
                  = std::chrono::high_resolution_clock::now();
              std::chrono::high_resolution_clock::duration duration
                  = currentTime - startTime;
              out_ << std::chrono::duration_cast<std::chrono::milliseconds>(
                          duration)
                          .count()
                   << " , " << goalMotions_[i]->cost.value() << ", "
                   << iterations_ << " , " << nn_->size() << " , "
                   << samplesGeneratedNum_ << std::endl;
            }
            bestCost_ = goalMotions_[i]->cost;
            updatedSolution = true;
          }

          sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
          if (sufficientlyShort)
          {
            solution = goalMotions_[i];
            break;
          }
          else if (
              !solution
              || opt_->isCostBetterThan(goalMotions_[i]->cost, solution->cost))
          {
            solution = goalMotions_[i];
            updatedSolution = true;
          }
        }

        if (updatedSolution)
        {
          if (useTreePruning_)
          {
            pruneTree(bestCost_);
          }

          if (intermediateSolutionCallback)
          {
            std::vector<const base::State*> spath;
            Motion* intermediate_solution
                = solution
                      ->parent; // Do not include goal state to simplify code.

            // Push back until we find the start, but not the start itself
            while (intermediate_solution->parent != nullptr)
            {
              spath.push_back(intermediate_solution->state);
              intermediate_solution = intermediate_solution->parent;
            }

            intermediateSolutionCallback(this, spath, bestCost_);
          }
        }
      }

      // Checking for approximate solution (closest state found to the goal)
      if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
      {
        OMPL_INFORM("%s: distanceFromGoal - %d , approximatedist - %d", 
                    getName().c_str(), distanceFromGoal, approximatedist);
        approximation = motion;
        approximatedist = distanceFromGoal;
      }
    }
    /*else
    {
        std::cout << "check motion failed " << std::endl;
    }*/

    // terminate if a sufficient solution is found
    if (solution && sufficientlyShort)
      break;
  }

  bool approximate = (solution == nullptr);
  bool addedSolution = false;
  if (approximate)
  {
    solution = approximation;
  }
  else
  {
    lastGoalMotion_ = solution;
  }

  if (solution != nullptr)
  {
    OMPL_INFORM("%s: solution found", getName().c_str());
    ptc.terminate();
    // construct the solution path
    std::vector<Motion*> mpath;
    while (solution != nullptr)
    {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    // set the solution path
    PathGeometric* geoPath = new PathGeometric(si_);
    for (int i = mpath.size() - 1; i >= 0; --i)
      geoPath->append(mpath[i]->state);

    base::PathPtr path(geoPath);
    // Add the solution path.
    base::PlannerSolution psol(path);
    psol.setPlannerName(getName());
    if (approximate)
      psol.setApproximate(approximatedist);
    // Does the solution satisfy the optimization objective?
    psol.setOptimized(opt_, bestCost_, sufficientlyShort);
    pdef_->addSolutionPath(psol);

    addedSolution = true;
  }

  si_->freeState(xstate);
  if (rmotion->state)
    si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM(
      "%s: Created %u new states. Checked %u rewire options. %u goal "
      "states in tree. Final solution cost %.3f",
      getName().c_str(),
      statesGenerated,
      rewireTest,
      goalMotions_.size(),
      bestCost_.value());

  return base::PlannerStatus(addedSolution, approximate);
}

::ompl::base::PathPtr MyInformedRRTstar::completeApproximateSolution(::ompl::base::PathPtr approximation)
{
  // convert approximation to a path geometric
  PathGeometric* geoPath = dynamic_cast<PathGeometric*>(approximation.get());
  ::ompl::base::PathPtr newPath = nullptr;

  // sample a goal state
  base::State* goalState = si_->allocState();
  base::Goal* goal = pdef_->getGoal().get();
  base::GoalSampleableRegion* goal_s
      = dynamic_cast<base::GoalSampleableRegion*>(goal);
  goal_s->sampleGoal(goalState);

  std::size_t geoPathLen = geoPath->getStateCount();
  if(geoPathLen>0)
  {
    base::State* lastState = geoPath->getState(geoPathLen-1);
    // try to connect to the goal state
    if(lastState && si_->checkMotion(lastState, goalState))
    {
      OMPL_INFORM("WE CAN COMPLETE THE APPROXIMATE SOLUTION");
      PathGeometric* newGeoPath = new PathGeometric(*geoPath);
      if (newGeoPath->getStateCount()!=geoPath->getStateCount())
      {
        std::cout << "AFTER CLONE, MISMATCH STATE COUNT" << std::endl;
      }
      newGeoPath->append(goalState);
      newPath = ::ompl::base::PathPtr(newGeoPath);
    }
  }

  return newPath;
}

bool MyInformedRRTstar::toState(
    std::string stateString, ompl::base::State* toState)
{
  if (toState == nullptr)
  {
    return false;
  }
  if (stateString == "")
  {
    return false;
  }
  std::stringstream iss(stateString);
  int dimIdx = 0;
  double val = 0;
  while (iss >> val && dimIdx < getSpaceInformation()->getStateDimension())
  {
    toState->as<ompl::base::RealVectorStateSpace::StateType>()->values[dimIdx]
        = val;
    dimIdx++;
  }

  return true;
}

std::string MyInformedRRTstar::fromState(ompl::base::State* fromState)
{
  std::stringstream oss;
  for (unsigned int dimIdx = 0;
       dimIdx < getSpaceInformation()->getStateDimension();
       ++dimIdx)
  {
    oss << fromState->as<ompl::base::RealVectorStateSpace::StateType>()
               ->values[dimIdx]
        << " ";
  }
  return oss.str();
}

}
}
