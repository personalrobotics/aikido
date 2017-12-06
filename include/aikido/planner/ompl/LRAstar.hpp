#ifndef AIKIDO_PLANNER_OMPL_LRASTAR_HPP_
#define AIKIDO_PLANNER_OMPL_LRASTAR_HPP_

#include <vector>
#include <string> 
#include <unordered_set>
#include <queue>
#include <exception>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include "util.hpp"

namespace aikido {
namespace planner {
namespace ompl {

/// The OMPL Planner class that implements the algorithm
class LRAstar: public ::ompl::base::Planner
{

public:

  // Properties associated with each roadmap vertex
  struct VProp
  {
    /// The underlying state of the vertex
    StateWrapperPtr v_state;

    /// Collision status of the vertex
    CollisionStatus vertexStatus;

    /// Flag to check if vertex is within the lazyband
    bool visited;

    /// Node Structure associated with vertex
    NodeEntry<size_t> node;
  }; // struct VProp

  struct EProp
  {
    /// The length of the edge using the space distance metric
    double length;

    /// Collision status of the edge
    CollisionStatus edgeStatus;

    /// States embedded in an edge
    std::vector< StateWrapperPtr > edgeStates;

  }; // struct EProp

  // Graph definitions
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VProp, EProp> Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
  typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
  typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

  // Boost Graph Property Map definitions
  // Vertex Maps
  typedef boost::property_map<Graph, boost::vertex_index_t VProp::*>::type VertexIndexMap;
  typedef boost::property_map<Graph, StateWrapperPtr VProp::*>::type VPStateMap;
  typedef boost::property_map<Graph, CollisionStatus VProp::*>::type VPStatusMap;
  typedef boost::property_map<Graph, bool VProp::*>::type VPVisitedMap;
  typedef boost::property_map<Graph, NodeEntry<Vertex> VProp::*>::type VPVertexNodeMap;

  // Edge Maps
  typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type EdgeIndexMap;
  typedef boost::property_map<Graph, CollisionStatus EProp::*>::type EPStatusMap;
  typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;


  // Algorithmic type definition
  struct HashFunction
  {
    std::size_t operator()(const Vertex& v) const
    {
      return v;
    }
  };

  typedef NodeEntry<Vertex> Node;
  typedef std::unordered_set< Vertex, HashFunction > unorderedSet;

  /// The pointer to the OMPL state space
  const ::ompl::base::StateSpacePtr mSpace;

  /// The fixed roadmpa over which the search is done
  Graph g;

  LRAstar(const ::ompl::base::SpaceInformationPtr &si);

  /// \param[in] si The OMPL space information manager
  /// \param[in] _roadmapFileName The path to the .graphml file that encodes the roadmap
  /// \param[in] _lookahead The lazy lookahead that defines the horizon
  /// \param[in] _greediness The greediness to evaluate lazy shortest paths
  LRAstar(const ::ompl::base::SpaceInformationPtr &si,
          const std::string& _roadmapFileName,
          double _lookahead,
          double _greediness);

  ~LRAstar(void);

  // Setters and Getters
  /// Set/Get Value of Lazy Lookahead
  void setLookahead(double _lookahead);
  double getLookahead() const;
  /// Set/Get Value of Greediness
  void setGreediness(double _greediness);
  double getGreediness() const;
  /// Set/Get Value of Connection Radius
  void setConnectionRadius(double _connectionRadius);
  double getConnectionRadius() const;
  /// Get ID of start vertex
  Vertex getStartVertex() const;
  /// Get ID of goal vertex
  Vertex getGoalVertex() const;
  /// Get the shortest path cost
  double getBestPathCost() const;
  /// Set/Get Roadmap Information
  void setRoadmapFileName(const std::string& _roadmapFileName);
  std::string getRoadmapFileName() const;

  // Internal Evaluation Methods
  /// Number of edges evaluated thus far
  inline unsigned int getNumEdgeEvals(){ return mNumEdgeEvals;}
  /// Number of calls to collision checker made thus far
  inline unsigned int getNumEdgeRewires(){ return mNumEdgeRewires;}
  /// Total time spent doing searches
  inline double getSearchTime(){ return mSearchTime;}
  /// Total time spent doing collision checks
  inline double getCollCheckTime(){return mCollCheckTime;}

  // OMPL required methods
  void setProblemDefinition(const ::ompl::base::ProblemDefinitionPtr &pdef);
  ::ompl::base::PlannerStatus solve(const ::ompl::base::PlannerTerminationCondition&);
  void setup();
  ::ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

  /// Given a new edge, initialize the embedded configurations
  /// along the edge using the resolution of the underlying space.
  /// This is called during problem setup.
  /// \param[in] e The edge ID to initialize with configurations
  void initializeEdgePoints(const Edge& e);

  /// Evaluate an edge to determine its collision status
  /// and assign it to the underlying property of the edge.
  /// \param[in] The edge ID to check for
  /// \return True if edge is valid
  bool evaluateEdge(const Edge& e);

  // Roadmap things
  boost::shared_ptr<RoadmapFromFile<Graph, VPStateMap, StateWrapper, EPLengthMap>> roadmapPtr;

private:

  // Planner helpers
  Vertex mStartVertex;
  Vertex mGoalVertex;
  unorderedSet mSetRewire;
  BisectPerm mBisectPermObj;

  // Planner parameters
  std::string mRoadmapFileName;
  double mLookahead;
  double mGreediness;
  double mConnectionRadius;
  double mBestPathCost;
  double mCheckRadius;

  // For planner evaluation
  unsigned int mNumEdgeEvals;
  unsigned int mNumEdgeRewires;
  double mSearchTime;
  double mCollCheckTime;

  // Supplementary Functions
  /// Find the path from leaf vertex to border vertex
  /// \param[in] _vertex Vertex on the frontier
  std::vector<Vertex> pathToBorder(Vertex _vertex);
  /// G-value of vertex
  /// \param[in] _vertex Vertex
  double estimateCostToCome(Vertex _vertex);
  /// Heursitic Function of vertex
  /// \param[in] _vertex Vertex
  double heuristicFunction(Vertex _vertex);
  /// F-value of vertex
  /// \param _vertex Vertex
  double estimateTotalCost(Vertex _vertex);

  // Main Algorithm Functions
  /// Extend the Lazy Band
  /// param[t]  queue with custom comparator
  /// param[in] qExtend priority queue of vertices to extend
  /// param[in] qFrontier priority queue of leaf vertices
  template<class TF>
  void extendLazyBand(TF &qExtend, TF &qFrontier);
  /// Update the Lazy Band
  /// param[t]  queue with custom comparator
  /// param[in] qUpdate priority queue of vertices to update
  /// param[in] qExtend priority queue of vertices to extend
  /// param[in] qFrontier priority queue of leaf vertices
  template<class TG, class TF>
  void updateLazyBand(TG &qUpdate, TF &qExtend, TF &qFrontier);
  /// Rewire the Lazy Band if Collision Encountered
  /// param[t]  queue with custom comparator
  /// param[in] qExtend priority queue of vertices to rewire
  /// param[in] qExtend priority queue of vertices to extend
  /// param[in] qFrontier priority queue of leaf vertices
  template<class TG, class TF>
  void rewireLazyBand(TG &qRewire, TF &qExtend, TF &qFrontier);
  /// Extend the Lazy Band
  /// param[t] TG queue with custom comparator
  /// param[in] pathTail sequence of edges to evaluate
  /// param[in] qExtend priority queue of vertices to update
  /// param[in] qExtend priority queue of vertices to rewire
  /// param[in] qExtend priority queue of vertices to extend
  /// param[in] qFrontier priority queue of leaf vertices
  template<class TG, class TF>
  bool evaluatePath(std::vector<Vertex> pathTail, TG &qUpdate, TG &qRewire, TF &qExtend, TF &qFrontier);

}; // class LRAstar

} // namespace ompl
} // namespace planner
} // namespace aikido

#endif // AIKIDO_PLANNER_OMPL_LRASTAR_HPP_
