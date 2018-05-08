#include <aikido/planner/kinodynamics/ompl/OmplHelpers.hpp>

//
// Function to convert a State to a VectorXd
//
// @param s Ompl State
// @return Eigen VectorXd
//
bool get_eigen_vector(const ompl::base::State* s, Eigen::VectorXd& vec)
{
  /*
  if(vec.SizeAtCompileTime != param.dimensions)
  {
      return false;
  }*/

  auto tmp = s->as<ompl::base::RealVectorStateSpace::StateType>();
  for (uint i = 0; i < param.dimensions; i++)
  {
    // vec[i] = s->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    vec[i] = tmp->values[i];
  }
  return true;
}

//
// Convert an Eigen::VectorXd to an ompl State pointer
//
// @param vec VectorXd representing the state
// @return Pointer to an ompl state
//
bool get_ompl_state(const Eigen::VectorXd& vec, ompl::base::State* state)
{
  for (uint i = 0; i < param.dimensions; i++)
  {
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]
        = vec[i];
  }

  return true;
}

void print_out_states(ompl::base::State* statePtr)
{
  double* val
      = static_cast<ompl::base::RealVectorStateSpace::StateType*>(statePtr)
            ->values;

  std::cout << "Printing sample of size: " << param.dimensions << " | Vec: [ ";
  for (uint i = 0; i < param.dimensions; i++)
  {
    std::cout << val[i] << " ";
  }
  std::cout << "]" << std::endl;
}

void print_out_states(const Eigen::VectorXd& state)
{
  std::cout << "[ ";
  for (uint i = 0; i < state.size(); i++)
  {
    std::cout << state[i] << " ";
  }
  std::cout << " ]" << std::endl;
}

bool approximate_cost(double a, double b)
{
  return std::fabs(a - b) < std::numeric_limits<double>::epsilon();
}
