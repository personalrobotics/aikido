#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <aikido/planner/kinodynamics/ompl/MyOptimizationObjective.hpp>

///
/// Function to convert a State to a VectorXd
///
/// @param s Ompl State
/// @return Eigen VectorXd
///
bool get_eigen_vector(const ompl::base::State* s, Eigen::VectorXd& vec);

///
/// Function to convert a std::vector to a VectorXd
///
/// @param vec Standard vector
/// @return A VectorXd with the state information
///
template <typename T>
Eigen::VectorXd get_eigen_vector(const std::vector<T>& vec)
{
  Eigen::VectorXd v(param.dimensions);

  for (uint i = 0; i < param.dimensions; i++)
  {
    v[i] = vec[i];
  }

  return v;
}

///
/// Convert an Eigen::VectorXd to an ompl State pointer
///
/// @param vec VectorXd representing the state
/// @return Pointer to an ompl state
///
bool get_ompl_state(const Eigen::VectorXd& vec, ompl::base::State* state);

void print_out_states(ompl::base::State* statePtr);

void print_out_states(const Eigen::VectorXd& state);

bool approximate_cost(double a, double b);
