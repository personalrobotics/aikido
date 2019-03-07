#include "aikido/planner/SequenceMetaPlanner.hpp"
#include <iostream>

namespace aikido {
namespace planner {

//==============================================================================
trajectory::TrajectoryPtr SequenceMetaPlanner::plan(
    const Problem& problem, Result* result)
{
  // for (const auto& planner : mPlanners)
  for(std::size_t i = 0 ; i < mPlanners.size(); ++i)
  {
  	const auto& planner = mPlanners[i];
    auto trajectory = planner->plan(problem, result);
    if (trajectory)
    {
    	std::cout << "Planner " << i << " succeeded" << std::endl;
      return trajectory;
    }
  }

  return nullptr;
}

} // namespace planner
} // namespace aikido
