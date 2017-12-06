#ifndef NODE_STRUCTURE_HPP_
#define NODE_STRUCTURE_HPP_

#include <vector>

namespace aikido {
namespace planner {
namespace ompl {

template <class VertexType>
class NodeEntry
{
  VertexType vertex_;
  VertexType parent_;
  std::vector<VertexType> children_;
  double cost_;
  double lazyCost_;
  double budget_;
  double heuristic_;

public:
  NodeEntry(VertexType vertex, VertexType parent, std::vector<VertexType> children, double c, double l, double b, double h):
      vertex_{vertex},
      parent_{parent},
      children_{children},
      cost_{c},
      lazyCost_{l},
      budget_{b},
      heuristic_{h}
      {}
  NodeEntry()
      {}

  // Access the class member variables
  VertexType vertex() const {return vertex_;}
  VertexType parent() const {return parent_;}
  std::vector<VertexType> children() const {return children_;}
  std::vector<VertexType>& children()  {return children_;}

  double cost() const {return cost_;}
  double lazyCost() const {return lazyCost_;}
  double budget() const {return budget_;}
  double heuristic() const {return heuristic_;}

  // Manipulate the Node
  void updateParent(VertexType p)
  {
    parent_ = p;
  }
  void updateCost(double cost)
  {
    cost_ = cost;
  }
  void updateLazyCost(double lazyCost)
  {
    lazyCost_ = lazyCost;
  }
  void updateBudget(double budget)
  {
    budget_ = budget;
  }
  void updateHeuristic(double heuristic)
  {
    heuristic_ = heuristic;
  }
};

} // nameplace ompl
} // nameplace planner
} // nameplace aikido

#endif
