#include <algorithm>        // std::reverse
#include <cmath>            // pow, sqrt
#include <iostream>         // std::cerr
#include <queue>            // std::priority_queue
#include <set>              // std::set
#include <unordered_set>    // std::unordered_set
#include <fstream>          // Write to file
#include <assert.h>         // Debug
#include <chrono>           // record rewireTime 

#include "aikido/planner/ompl/LRAstar.hpp"

namespace aikido {
namespace planner {
namespace ompl {

LRAstar::LRAstar(const ::ompl::base::SpaceInformationPtr &si)
  : ::ompl::base::Planner(si, "LRAstar")
  , mSpace(si->getStateSpace())
  , mRoadmapFileName("")
  , mLookahead(1.0)
  , mGreediness(1.0)
  , mConnectionRadius(1.0)
  , mBestPathCost(std::numeric_limits<double>::infinity())
  , mCheckRadius(0.5*mSpace->getLongestValidSegmentLength())
  , mNumEdgeEvals(0u)
  , mNumEdgeRewires(0u)
  , mSearchTime(0.0)
  , mCollCheckTime(0.0)
{
  Planner::declareParam<double>("lookahead", this, &LRAstar::setLookahead, &LRAstar::getLookahead);
  Planner::declareParam<double>("greediness", this, &LRAstar::setGreediness, &LRAstar::getGreediness);
  Planner::declareParam<std::string>("roadmap_filename", this, &LRAstar::setRoadmapFileName, &LRAstar::getRoadmapFileName);
}

LRAstar::LRAstar(const ::ompl::base::SpaceInformationPtr &si,
  const std::string& _roadmapFileName,
  double _lookahead,
  double _greediness)
  : ::ompl::base::Planner(si, "LRAstar")
  , mSpace(si->getStateSpace())
  , mRoadmapFileName(_roadmapFileName)
  , mLookahead(_lookahead)
  , mGreediness(_greediness)
  , mConnectionRadius(1.0)
  , mBestPathCost(std::numeric_limits<double>::infinity())
  , mCheckRadius(0.5*mSpace->getLongestValidSegmentLength())
  , mNumEdgeEvals(0u)
  , mNumEdgeRewires(0u)
  , mSearchTime(0.0)
  , mCollCheckTime(0.0)
{
}

LRAstar::~LRAstar()
{
}

// //////////////////////////////////////////////////////////////////////
// Setters
void LRAstar::setLookahead(double _lookahead)
{
  // OMPL_INFORM("Lookahead Set: %d", _lookahead);
  mLookahead = _lookahead;
}

void LRAstar::setGreediness(double _greediness)
{
  // OMPL_INFORM("Greediness Set: %d", _greediness);
  mGreediness = _greediness;
}

void LRAstar::setRoadmapFileName(const std::string& _roadmapFileName)
{
  mRoadmapFileName = _roadmapFileName;
}

void LRAstar::setConnectionRadius(double _connectionRadius)
{
  // OMPL_INFORM("Connection Radius Set: %d", _connectionRadius);
  mConnectionRadius = _connectionRadius;
}

// Getters
double LRAstar::getLookahead() const
{
  return mLookahead;
}

double LRAstar::getGreediness() const
{
  return mGreediness;
}

double LRAstar::getConnectionRadius() const
{
  return mConnectionRadius;
}

LRAstar::Vertex LRAstar::getStartVertex() const
{
  return mStartVertex;
}

LRAstar::Vertex LRAstar::getGoalVertex() const
{
  return mGoalVertex;
}

double LRAstar::getBestPathCost() const
{
  return mBestPathCost;
}

std::string LRAstar::getRoadmapFileName() const
{
  return mRoadmapFileName;
}


//////////////////////////////////////////////////////////////////////
// Public Helper Methods

::ompl::base::PathPtr LRAstar::constructSolution(const Vertex &start, const Vertex &goal)
{
  std::set<Vertex> seen;

  ::ompl::geometric::PathGeometric *path = new ::ompl::geometric::PathGeometric(si_);
  Vertex v = goal;
  while (v != start)
  {      
    if (seen.find(v) != seen.end())
    {
      // OMPL_ERROR("infinite loop");
      break;
    }

    seen.insert(v);
    path->append(g[v].v_state->state);
    v = g[v].node.parent();
  }

  if (v == start)
  {
    path->append(g[start].v_state->state);
  }
  path->reverse();
  return ::ompl::base::PathPtr(path);
}

void LRAstar::initializeEdgePoints(const Edge& e)
{
  auto startState = g[source(e,g)].v_state->state;
  auto endState = g[target(e,g)].v_state->state;

  unsigned int nStates = static_cast<unsigned int>(std::floor(g[e].length / (2.0*mCheckRadius)));
  
  // Just start and goal
  if(nStates < 2u) 
  {
    nStates = 2u;
  }

  g[e].edgeStates.resize(nStates);

  for(unsigned int i = 0; i < nStates; i++)
  {
    g[e].edgeStates[i].reset(new StateWrapper(mSpace));
  }

  const std::vector< std::pair<int,int> > & order = mBisectPermObj.get(nStates);

  for(unsigned int i = 0; i < nStates; i++)
  {
    mSpace->interpolate(startState, endState,
      1.0*(1+order[i].first)/(nStates+1), g[e].edgeStates[i]->state);
  }
}

bool LRAstar::evaluateEdge(const LRAstar::Edge& e)
{
  // March along edge states with highest resolution
  mNumEdgeEvals++;

  auto validityChecker = si_->getStateValidityChecker();
  
  Vertex startVertex = source(e,g);
  Vertex endVertex   = target(e,g);
  auto startState = g[startVertex].v_state->state;
  auto endState = g[endVertex].v_state->state;

  auto nStates = g[e].edgeStates.size();

  bool checkResult;
  std::chrono::time_point<std::chrono::system_clock> startEvaluationTime{std::chrono::system_clock::now()};
  
  // Evaluate Start and End States [we only assume states in self-collision are pruned out]
  checkResult = validityChecker->isValid(startState);
  if(!checkResult)
  {
    g[startVertex].vertexStatus = CollisionStatus::BLOCKED;
    return checkResult;
  }
  checkResult = validityChecker->isValid(endState);
  if(!checkResult)
  {
    g[endVertex].vertexStatus = CollisionStatus::BLOCKED;
    return checkResult;
  }

  // Evaluate the States in between
  for(unsigned int i = 1; i < nStates-1; i++)
  {
    checkResult = validityChecker->isValid(g[e].edgeStates[i]->state);
    if(!checkResult)
    {
      g[e].edgeStatus = CollisionStatus::BLOCKED;
      g[e].length = std::numeric_limits<double>::max();
      break;
    }
  }
  
  std::chrono::time_point<std::chrono::system_clock> endEvaluationTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endEvaluationTime-startEvaluationTime};
  mCollCheckTime += elapsedSeconds.count();

  return checkResult;
}

//////////////////////////////////////////////////////////////////////
// Private Helper Methods

/// Supplementary Functions
using Vertex = LRAstar::Vertex;
std::vector<Vertex> LRAstar::pathToBorder(Vertex v)
{
  std::vector<Vertex> path;
  path.emplace_back(v);
  double currentBudget = g[v].node.budget();

  while(currentBudget > 0)
  {
    v = g[v].node.parent();
    assert(g[v].node.budget() == currentBudget - 1);
    path.emplace_back(v);
    currentBudget = g[v].node.budget();

    assert(g[v].visited);
    assert(g[v].node.parent() != v);
    assert(g[v].node.budget() <= mLookahead);
  }
  return path;
}

double LRAstar::estimateCostToCome(Vertex v)
{
  return g[v].node.cost() + g[v].node.lazyCost();
}

double LRAstar::heuristicFunction(Vertex v)
{
  return mSpace->distance(g[v].v_state->state, g[mGoalVertex].v_state->state);
}

double LRAstar::estimateTotalCost(Vertex v)
{
  return estimateCostToCome(v) + heuristicFunction(v);
}


/// Main Functions
template<class TF>
void LRAstar::extendLazyBand(TF &qExtend, TF &qFrontier)
{
  while(!qExtend.empty())
  {
    // Obtain the Top Key in qFrontier for Lazy Extension
    double cReference;
    if(qFrontier.empty())
      cReference = std::numeric_limits<double>::max();
    else
    {
      Vertex vReference = *qFrontier.begin();
      cReference = estimateTotalCost(vReference);
    }
    Vertex u = *qExtend.begin();

    // Lazy Extension
    if(estimateTotalCost(u) >= cReference)
      break;

    qExtend.erase(qExtend.begin());
    assert(g[u].node.budget() < mLookahead); 
    assert(g[u].visited);

    if(g[u].vertexStatus == CollisionStatus::BLOCKED)
      continue;

    if(u == mGoalVertex)
      qFrontier.emplace(u);

    else
    {
      NeighborIter ni, ni_end;
      for (boost::tie(ni, ni_end) = adjacent_vertices(u, g); ni != ni_end; ++ni)
      {
        Vertex v = *ni;

        if(g[v].vertexStatus == CollisionStatus::BLOCKED)
          continue;

        // Enforce prevention of loops
        if(v == g[u].node.parent())
            continue;

        // Determine the edge length
        Edge uv;
        bool edgeExists;
        boost::tie(uv, edgeExists) = edge(u, v, g);
        assert(edgeExists);
        double edgeLength = g[uv].length;

        if(g[uv].edgeStatus == CollisionStatus::FREE)
        {
          Node nv = {v, u, {}, g[u].node.cost(), g[u].node.lazyCost() + edgeLength, g[u].node.budget() + 1, heuristicFunction(v)};
          if(g[v].visited == false)
          {
            g[v].visited = true; 
            assert(qExtend.find(v) == qExtend.end());
            assert(qFrontier.find(v) == qFrontier.end());
          }
          else
          {
            double estimateOld = estimateCostToCome(v);
            double estimateNew = estimateCostToCome(u) + edgeLength;
            Vertex previousParent = g[v].node.parent();

            if(estimateOld < estimateNew)
              continue;

            // Tie-Breaking Rule
            if(estimateOld == estimateNew)
            {
              if(previousParent < u)
                continue;
            }

            // else, update the old parent and the subsequent subtree

            // Remove vertex from its current siblings
            std::vector<Vertex>& children = g[previousParent].node.children();
            std::size_t indx;
            for(indx = 0; indx != children.size(); ++indx)
            {
              if(children[indx] == v)
                break;
            }
            assert(indx != children.size()); //child has been found

            // Remove child 
            if (indx != children.size()-1)
              children[indx] = children.back();
            children.pop_back();

            // Remove old node from any queue
            auto iterQ = qFrontier.find(v);
            if(iterQ != qFrontier.end())
              qFrontier.erase(iterQ);

            iterQ = qExtend.find(v);
            if(iterQ != qExtend.end())
              qExtend.erase(iterQ);


            // If the budget of "v" is alpha, mark entire subsequent subtree as not visited
            // These nodes don't need rewiring since shortest path to them must be via v
            std::vector<Vertex> subtree = {v};
            while(!subtree.empty())
            {
              auto iterT = subtree.rbegin();
              std::vector<Vertex>& children = g[*iterT].node.children();    
              subtree.pop_back();
              
              for(auto iterV = children.begin(); iterV != children.end(); ++iterV)
              {
                g[*iterV].visited = false;
                subtree.emplace_back(*iterV);

                auto iterQ = qFrontier.find(*iterV);
                if(iterQ != qFrontier.end())
                  qFrontier.erase(iterQ);

                iterQ = qExtend.find(*iterV);
                if(iterQ != qExtend.end())
                  qExtend.erase(iterQ);
              }
              children.clear();
            }
          }

          // Update Vertex Node
          g[v].node = nv;

          // Add it to its new siblings
          std::vector<Vertex>& children = g[u].node.children();
          children.emplace_back(v);

          // Add it to appropriate queue
          double budget = g[v].node.budget();
          if (budget == mLookahead)
          {
            assert(qFrontier.find(v) == qFrontier.end());
            qFrontier.emplace(v);
          }
          else // budget < mLookahead
          {
            assert(qExtend.find(v) == qExtend.end());
            qExtend.emplace(v);
          }
        }
      }
    }
  }
}

template<class TG, class TF>
void LRAstar::updateLazyBand(TG &qUpdate, TF &qExtend, TF &qFrontier)
{
  while(!qUpdate.empty())
  {
    Vertex u = *qUpdate.begin();
    qUpdate.erase(qUpdate.begin());

    assert(g[u].node.budget() < mLookahead);
    bool isLeaf;

    std::vector<Vertex>& children = g[u].node.children();

    for(auto iterV = children.begin(); iterV != children.end(); ++iterV)
    {
      Vertex v = *iterV;
      Node nv = g[v].node;
      isLeaf = false;

      if(nv.budget() != 0)
      {
        Edge uv;
        bool edgeExists;
        boost::tie(uv, edgeExists) = edge(u, v, g);
        assert(edgeExists);
        double edgeLength = g[uv].length;

        // Remove vertex from qFrontier
        if(nv.budget() == mLookahead)
        {
          auto iterQ = qFrontier.find(v);
          if(iterQ != qFrontier.end())
            qFrontier.erase(iterQ);
          isLeaf = true;
        }

        // Update nodes in qExtend
        auto iterQ = qExtend.find(v);
        if(iterQ != qExtend.end())
        {
          qExtend.erase(iterQ);
          isLeaf = true;
        }

        nv.updateCost(g[u].node.cost());
        nv.updateLazyCost(g[u].node.lazyCost() + edgeLength);
        nv.updateBudget(g[u].node.budget() + 1);
        g[v].node = nv;

        if(isLeaf || v == mGoalVertex)
        {
          assert(qExtend.find(u) == qExtend.end());
          assert(qFrontier.find(u) == qFrontier.end());
          assert(qExtend.find(v) == qExtend.end());
          qExtend.emplace(v);
        }
        assert(g[v].node.budget() < mLookahead);

        qUpdate.emplace(v);
      }
    }
  }
}

template<class TG, class TF>
void LRAstar::rewireLazyBand(TG &qRewire, TF &qExtend, TF &qFrontier)
{
  assert(mSetRewire.empty());

  // 1. Collect all the nodes that need to be rewired
  while(!qRewire.empty())
  {
    Vertex v = *qRewire.begin();
    qRewire.erase(qRewire.begin());

    // Add all the children of the current node to qRewire and empty the children vector
    std::vector<Vertex>& children = g[v].node.children();
    for(auto iterV = children.begin(); iterV != children.end(); ++iterV)
    {
      qRewire.emplace(*iterV);
    }
    children.clear();

    // Add the vertex to set
    mSetRewire.insert(v);

    // Remove from qFrontier
    auto iterQ = qFrontier.find(v);
    if(iterQ != qFrontier.end())
      qFrontier.erase(iterQ);
    assert(qFrontier.find(v) == qFrontier.end());

    // Remove from qExtend
    iterQ = qExtend.find(v);
    if(iterQ != qExtend.end())
      qExtend.erase(iterQ);
    assert(qExtend.find(v) == qExtend.end());

    // Assign default values
    g[v].node.updateParent(v);
    g[v].node.updateCost(std::numeric_limits<double>::max());
    g[v].node.updateLazyCost(0);
    g[v].node.updateBudget(0);

    // Mark it as not visited
    g[v].visited = false;  
  }

  // Record number of edge rewires
  mNumEdgeRewires += mSetRewire.size();

  // 2. Assign the nodes keys
  for(auto iterS = mSetRewire.begin(); iterS != mSetRewire.end(); ++iterS)
  {
    Vertex v = *iterS;
    if(g[v].vertexStatus == CollisionStatus::BLOCKED)
      continue;

    NeighborIter ni, ni_end;
    for(boost::tie(ni, ni_end) = adjacent_vertices(v, g); ni != ni_end; ++ni)
    {
      Vertex u = *ni; // Possible parent

      if(g[u].vertexStatus == CollisionStatus::BLOCKED)
        continue;

      if(u == mGoalVertex) // Do not rewire to goal vertex
        continue;
      
      if(g[u].node.cost() == std::numeric_limits<double>::max())
        continue;

      if (g[u].visited == false)
        continue;

      if(g[u].node.budget() == mLookahead) // parent should have budget
        continue;

      if(qExtend.find(u) != qExtend.end())
        continue;

      assert (mSetRewire.find(u) == mSetRewire.end());
      assert(v != g[u].node.parent()); // entire subtree should have been collected

      Edge uv;
      bool edgeExists;
      boost::tie(uv, edgeExists) = edge(u, v, g);
      assert(edgeExists);
      double edgeLength = g[uv].length;
      
      if(g[uv].edgeStatus == CollisionStatus::FREE)
      {
        if(estimateCostToCome(v) > estimateCostToCome(u) + edgeLength ||
          (estimateCostToCome(v) == estimateCostToCome(u) + edgeLength && u < g[v].node.parent()))
        {
          g[v].node.updateCost(g[u].node.cost());
          g[v].node.updateLazyCost(g[u].node.lazyCost() + edgeLength);
          g[v].node.updateParent(u);
          g[v].node.updateBudget(g[u].node.budget() + 1);
        }
      }
    }
    qRewire.emplace(v);
  }

  // Obtain the Top Key in qFrontier for Lazy Rewire
  double cReference;
  if(qFrontier.empty())
    cReference = std::numeric_limits<double>::max();
  else
  {
    Vertex vReference = *qFrontier.begin();
    cReference = estimateTotalCost(vReference);
  }

  // 3. Start Rewiring in the cost space
  while(!qRewire.empty())
  {
    Vertex u = *qRewire.begin();
    qRewire.erase(qRewire.begin());

    if(u == g[u].node.parent())
      continue;

    if(estimateTotalCost(g[u].node.parent()) >= cReference)
    {
      qExtend.emplace(g[u].node.parent());
      continue;
    }

    // Since valid parent is found, mark as visited
    g[u].visited = true;

    // Let the parent know of its new child
    Vertex p = g[u].node.parent();
    std::vector<Vertex>& children = g[p].node.children();
    children.emplace_back(u);

    if(g[u].node.budget() < mLookahead && u != mGoalVertex)
    {
      assert(qExtend.find(u) == qExtend.end());
      assert(qExtend.find(p) == qExtend.end());
      assert(g[u].node.children().empty());

      qExtend.emplace(u);
    }

    if(g[u].node.budget() == mLookahead || u == mGoalVertex)
    {
      assert(qFrontier.find(u) == qFrontier.end());
      qFrontier.emplace(u);
      continue;
    }

    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(u, g); ni != ni_end; ++ni)
    {
      Vertex v = *ni;

      if(g[v].vertexStatus == CollisionStatus::BLOCKED)
        continue;

      // Vertex needs to be in set to update
      if(qRewire.find(v) == qRewire.end())
        continue;

      assert(v != p); // Enforce prevention of loops - parent should not be in Qrewire
      
      Edge uv;
      bool edgeExists;
      boost::tie(uv, edgeExists) = edge(u, v, g);
      assert(edgeExists);
      double edgeLength = g[uv].length;

      if(g[uv].edgeStatus == CollisionStatus::FREE)
      {
        if(estimateCostToCome(v) > estimateCostToCome(u) + edgeLength ||
          (estimateCostToCome(v) == estimateCostToCome(u) + edgeLength && u < g[v].node.parent()))
        {
          if(qExtend.find(u) != qExtend.end())
          {
            qRewire.erase(v);
            g[v].visited = false;
            g[v].node.updateCost(std::numeric_limits<double>::max());
            g[v].node.updateParent(v);
            qRewire.emplace(v);
            continue;
          }

          qRewire.erase(v);

          g[v].node.updateCost(g[u].node.cost());
          g[v].node.updateLazyCost(g[u].node.lazyCost() + edgeLength);
          g[v].node.updateParent(u);
          g[v].node.updateBudget(g[u].node.budget() + 1);

          assert(g[u].node.budget() <  mLookahead);
          assert(g[v].node.budget() <= mLookahead);

          qRewire.emplace(v);
        }
      }
    }
  }
  mSetRewire.clear();
}

template<class TG, class TF>
bool LRAstar::evaluatePath(std::vector<Vertex> path, TG &qUpdate, TG &qRewire, TF &qExtend, TF &qFrontier)
{
  assert(path.size() != 1);

  // Increase beta to alpha if goal is on current path
  int greediness;
  if(path[0] == mGoalVertex)
    greediness = mLookahead;
  else
    greediness = mGreediness;
  
  bool isLeaf;
  for(auto iterV = path.rbegin(); iterV != path.rbegin() + greediness && iterV != path.rend(); ++iterV)
  {
    isLeaf = false;

    Vertex u = *iterV;
    Vertex v = *(iterV + 1);

    // Determine the edge length
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(u, v, g);
    assert(edgeExists);
    double edgeLength = g[uv].length;

    // Actual Collision Check
    if (!evaluateEdge(uv))
    {
      std::vector<Vertex>& children = g[u].node.children();
      std::size_t indx;
      for(indx = 0; indx != children.size(); ++indx)
      {
        if(children[indx] == v)
          break;
      }
      assert(indx != children.size()); // child has been found

      // Remove child 
      if (indx != children.size()-1)
          children[indx] = children.back();
      children.pop_back();

      qRewire.emplace(v);
      break;   
    }

    else
    {
      if(g[v].node.budget() == mLookahead)
      {
        auto iterQ = qFrontier.find(v);
        if(iterQ != qFrontier.end())
          qFrontier.erase(iterQ);
        isLeaf = true;
      }

      g[v].node.updateBudget(0);
      g[v].node.updateLazyCost(0);
      g[v].node.updateCost(g[u].node.cost() + edgeLength);

      assert(qFrontier.find(v) == qFrontier.end());
      assert(qExtend.find(v) == qExtend.end());

      if(isLeaf)
      {
        assert(qExtend.find(u) == qExtend.end());
        assert(qFrontier.find(u) == qFrontier.end());

        assert(qExtend.find(v) == qExtend.end());
        qExtend.emplace(v);
      }

      qUpdate.emplace(v);
      if(v == mGoalVertex)
        return true;
    }
  }
  return false;
}


// OMPL Methods
void LRAstar::setProblemDefinition(const ::ompl::base::ProblemDefinitionPtr &pdef)
{
  ::ompl::base::Planner::setProblemDefinition(pdef);

  StateWrapperPtr startState(new StateWrapper(mSpace));
  mSpace->copyState(startState->state, pdef_->getStartState(0));

  StateWrapperPtr goalState(new StateWrapper(mSpace));
  mSpace->copyState(goalState->state, pdef_->getGoal()->as<::ompl::base::GoalState>()->getState());

  auto validityChecker = si_->getStateValidityChecker();

  if(!validityChecker->isValid(startState->state))
    throw ::ompl::Exception("Start configuration is in collision!");
  if(!validityChecker->isValid(goalState->state))
    throw ::ompl::Exception("Goal configuration is in collision!");

  // Add start and goal vertices to the graph
  mStartVertex = boost::add_vertex(g);
  g[mStartVertex].v_state = startState;
  g[mStartVertex].vertexStatus = CollisionStatus::FREE;
  g[mStartVertex].visited = false;
  Node* nodeS = new Node();
  g[mStartVertex].node = *nodeS;

  mGoalVertex = boost::add_vertex(g);
  g[mGoalVertex].v_state = goalState;
  g[mGoalVertex].vertexStatus = CollisionStatus::FREE;
  g[mGoalVertex].visited = false;
  Node* nodeG = new Node();
  g[mGoalVertex].node = *nodeG;

  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi)
  {
    double startDist = mSpace->distance(g[*vi].v_state->state, startState->state);
    double goalDist  = mSpace->distance(g[*vi].v_state->state, goalState->state);

    if (startDist < mConnectionRadius)
    {
      if(mStartVertex == *vi)
        continue;
      std::pair<LRAstar::Edge,bool> newEdge = boost::add_edge(mStartVertex, *vi, g);
      g[newEdge.first].length = startDist;
      g[newEdge.first].edgeStatus = CollisionStatus::FREE;
      initializeEdgePoints(newEdge.first);
    }
    if (goalDist < mConnectionRadius)
    {
      if(mGoalVertex == *vi)
        continue;
      std::pair<LRAstar::Edge,bool> newEdge = boost::add_edge(mGoalVertex, *vi, g);
      g[newEdge.first].length = goalDist;
      g[newEdge.first].edgeStatus = CollisionStatus::FREE;
      initializeEdgePoints(newEdge.first);
    }
  }
}

void LRAstar::setup()
{
  ::ompl::base::Planner::setup();

  if(mRoadmapFileName == "")
    throw ::ompl::Exception("Roadmap name must be set!");

  roadmapPtr = boost::shared_ptr<RoadmapFromFile<Graph, VPStateMap, StateWrapper, EPLengthMap>>
                (new RoadmapFromFile<Graph, VPStateMap, StateWrapper, EPLengthMap>
                (mSpace, mRoadmapFileName));

  roadmapPtr->generate(g, get(&VProp::v_state, g),
                          get(&EProp::length, g));

  VPVisitedMap visitedMap = get(&VProp::visited, g);
  VPStatusMap vertexStatusMap = get(&VProp::vertexStatus, g);
  VPVertexNodeMap nodeMap = get(&VProp::node, g);
  EPStatusMap edgeStatusMap = get(&EProp::edgeStatus, g);

  // Set Default Values
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi)
  {
    visitedMap[*vi] = false;
    vertexStatusMap[*vi] = CollisionStatus::FREE;
    Node* node = new Node();
    nodeMap[*vi] = *node;
  }

  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
  {
    edgeStatusMap[*ei] = CollisionStatus::FREE;
    initializeEdgePoints(*ei);
  }
}

::ompl::base::PlannerStatus LRAstar::solve(const ::ompl::base::PlannerTerminationCondition&)
{

  // Priority Function: g-value
  auto cmpGValue = [&](Vertex left, Vertex right)
  {
      double estimateLeft = estimateCostToCome(left);
      double estimateRight = estimateCostToCome(right);

      if (estimateRight - estimateLeft > 0)
          return true;
      if (estimateLeft - estimateRight > 0)
          return false;
      if (left < right)
          return true;
      else
          return false; 
  };

  // Priority Function: f-value
  auto cmpFValue = [&](Vertex left, Vertex right)
  {
      double estimateLeft = estimateTotalCost(left);
      double estimateRight = estimateTotalCost(right);

      if (estimateRight - estimateLeft > 0)
          return true;
      if (estimateLeft - estimateRight > 0)
          return false;
      if (left < right)
          return true;
      else
          return false; 
  };

  std::set<Vertex, decltype(cmpGValue)> qUpdate(cmpGValue);
  std::set<Vertex, decltype(cmpGValue)> qRewire(cmpGValue);
  std::set<Vertex, decltype(cmpFValue)> qExtend(cmpFValue);
  std::set<Vertex, decltype(cmpFValue)> qFrontier(cmpFValue);

  bool solutionFound = false;

  // Track the running time of the algorithm
  std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};
  if(mStartVertex == mGoalVertex)
  {
      // OMPL_INFORM("Solution Found!");
      solutionFound = true;        
  }

  Node nStart(mStartVertex, -1, {}, 0.0, 0.0, 0.0, heuristicFunction(mStartVertex));
  g[mStartVertex].node = nStart;
  g[mStartVertex].visited = true;
  qExtend.insert(mStartVertex);

  extendLazyBand(qExtend, qFrontier);

  std::vector<Vertex> path;
  while((!qFrontier.empty() && !solutionFound) || !qExtend.empty())
  {
    Vertex vTop = *qFrontier.begin();
    qFrontier.erase(qFrontier.begin());
    assert(g[vTop].node.budget() == mLookahead || vTop == mGoalVertex);

    path = pathToBorder(vTop);
    bool goalFound = evaluatePath(path, qUpdate, qRewire, qExtend, qFrontier);

    if(goalFound)
    {
        // OMPL_INFORM("Solution Found!");
        solutionFound = true;
        break;
    }
    updateLazyBand(qUpdate, qExtend, qFrontier);
    rewireLazyBand(qRewire, qExtend, qFrontier);
    extendLazyBand(qExtend, qFrontier);
  }

  std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endTime-startTime};
  mSearchTime = elapsedSeconds.count() - mCollCheckTime;

  if(solutionFound)
  {
    pdef_->addSolutionPath(constructSolution(mStartVertex, mGoalVertex));
    return ::ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
  
  else
    return ::ompl::base::PlannerStatus::TIMEOUT;
    // OMPL_INFORM("Solution NOT Found");
}

} // namespace ompl
} // namespace planner
} // namespace aikido
